// use futures::{Stream, StreamExt};
use k::nalgebra::Quaternion;
use k::prelude::InverseKinematicsSolver;
use k::Chain;
use k::{Isometry3, Translation3, UnitQuaternion};
// use micro_sp::ToSPValue;
use micro_sp::*;
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::Header;
// use r2r::tf2_msgs::msg::TFMessage;
// use r2r::ur_script_msgs::srv::DashboardCommand as DBCommand;
use r2r::QosProfile;
use r2r_teaching_markers::TeachingMarkerServer;
// use r2r_transforms::*;
// use std::arch::global_asm;
use std::fs::File;
use std::io::Write;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};
use tempfile::tempdir;
use tokio::sync::{mpsc, oneshot};

use crate::*;

pub const UR_CONTROL_GHOST_TICKER_RATE: u64 = 20;
pub static DEFAULT_GHOST_BASEFRAME_ID: &'static str = "ghost_base_link"; // base_link if simulation, base if real or ursim
pub static DEFAULT_GHOST_FACEPLATE_ID: &'static str = "ghost_flange";
pub static DEFAULT_GHOST_TCP_ID: &'static str = "ghost_tool0";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn control_ghost(
    robot_name: String,
    // initial_faceplate_id: String,
    // initial_tcp_id: String,
    urdf: String,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>,
    global_transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>, // has to be global
) -> Result<(), Box<dyn std::error::Error>> {
    // so that the ghosts knows how to calculate inverse kinematics
    let current_face_plate = Arc::new(Mutex::new(DEFAULT_GHOST_FACEPLATE_ID.to_string()));

    // so that the ghosts knows how to calculate inverse kinematics
    let current_tcp = Arc::new(Mutex::new(DEFAULT_GHOST_TCP_ID.to_string()));

    let mut timer =
        arc_node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(
                UR_CONTROL_GHOST_TICKER_RATE,
            ))?;

    let ghost_state_publisher = arc_node
        .lock()
        .unwrap()
        .create_publisher::<JointState>("ghost_joint_states", QosProfile::default())?;

    let robot_name_clone = robot_name.clone();
    r2r::log_info!(
        &format!("{robot_name_clone}_control_ghost"),
        "Starting control ghost."
    );

    // Make a manipulatable kinematic chain using the urdf
    let (chain, joints, links) = chain_from_urdf_raw(robot_name.clone(), urdf).await;

    // let robot_name_clone = robot_name.clone();
    r2r::log_info!(
        &format!("{robot_name_clone}_control_ghost"),
        "Ghost initial state waiting for the main robot to spawn..."
    );

    let mut initial_joint_position_value = vec![];

    loop {
        let (response_tx, response_rx) = oneshot::channel();
        state_mgmt
            .send(StateManagement::Get((
                format!("{robot_name_clone}_joint_states"),
                response_tx,
            )))
            .await?;

        let joint_state = response_rx.await?;

        // Match on `joint_state` to see if it's our desired `Array(SPValueType::Float64, ...)`
        match joint_state {
            SPValue::Array(SPValueType::Float64, joints) => {
                // Try to collect all Float64 values into a Vec<f64>
                let mut temp = Vec::new();
                let mut valid = true;

                for val in &joints {
                    if let SPValue::Float64(num) = val {
                        temp.push(num.into_inner());
                    } else {
                        // If we hit anything that's not SPValue::Float64, skip this iteration
                        valid = false;
                        break;
                    }
                }

                // If all array elements were Float64, store them and break out
                if valid {
                    initial_joint_position_value = temp;
                    break;
                }
            }
            // If it’s not an array of Float64, just continue
            _ => {}
        }

        // Wait a bit and then try again
        timer.tick().await?;
    }

    r2r::log_info!(
        &format!("{robot_name_clone}_control_ghost"),
        "Ghost's initial state is now taken from the actual robot."
    );
    println!("Current Joint state: {:?}", initial_joint_position_value);
    // did we get what we expected
    r2r::log_info!(
        &format!("{robot_name_clone}_control_ghost"),
        "Found joints: {:?}",
        joints
    );
    r2r::log_info!(
        &format!("{robot_name_clone}_control_ghost"),
        "Found links: {:?}",
        links
    );

    // println!("{:?}", chain);

    let initial_joint_value = JointState {
        header: Header {
            ..Default::default()
        },
        name: joints.clone(),
        position: initial_joint_position_value,
        ..Default::default()
    };

    let joint_states = Arc::new(Mutex::new(initial_joint_value));

    // Make a teaching marker server
    let arc_node_clone = arc_node.clone();
    let teaching_marker_server = TeachingMarkerServer::new("ghost_markers", arc_node_clone);

    let arc_node_clone = arc_node.clone();
    teaching_marker_server.insert(
        format!("{robot_name}_ghost_marker").to_string(),
        DEFAULT_GHOST_TCP_ID.to_string(), // ??? here  of where?
        None,
        arc_node_clone,
    );

    // let static_tf_listener = arc_node
    //     .lock()
    //     .unwrap()
    //     .subscribe::<TFMessage>("tf_static", QosProfile::volatile(QosProfile::default()))
    //     .expect("Failed to initialize static_tf_listener.");

    let robot_name_string = robot_name.to_string();
    let robot_name_string_clone = robot_name_string.clone();
    let robot_name_string_clone_clone = robot_name_string.clone();
    let joint_states_clone = joint_states.clone();
    tokio::task::spawn(async move {
        match ghost_publisher_callback(
            robot_name_string_clone,
            ghost_state_publisher,
            timer,
            &joint_states_clone,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(
                &format!("{robot_name_string_clone_clone}_ghost_marker"),
                "Joint state publisher failed with: '{}'.",
                e
            ),
        };
    });

    let mut pose_update_timer =
        arc_node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(
                UR_CONTROL_GHOST_TICKER_RATE,
            ))?;

    // spawn a tokio task to listen to incomming teaching marker poses
    let joint_state_clone = joint_states.clone();
    let arc_chain = Arc::new(Mutex::new(chain));
    let arc_chain_clone = arc_chain.clone();
    let global_transform_buffer_clone = global_transform_buffer.clone();
    // let robot_name_string_clone = robot_name_string.clone();
    let robot_name_clone = robot_name.clone();
    let current_face_plate_clone = current_face_plate.clone();
    let current_tcp_clone = current_tcp.clone();
    tokio::task::spawn(async move {
        match pose_update(
            robot_name_clone.to_string(),
            &arc_chain_clone,
            &current_face_plate_clone,
            &current_tcp_clone,
            &joint_state_clone,
            &global_transform_buffer_clone,
            pose_update_timer,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(
                &format!("{robot_name_clone}_ghost_marker"),
                "Teaching mode subscriber failed with {}.",
                e
            ),
        };
    });

    let global_transform_buffer_clone = global_transform_buffer.clone();
    let robot_name_clone = robot_name.clone();
    let arc_chain_clone = arc_chain.clone();
    let current_face_plate_clone = current_face_plate.clone();
    let current_tcp_clone = current_tcp.clone();
    update_kinematic_chain(
        robot_name,
        &global_transform_buffer_clone,
        &arc_chain_clone,
        &current_face_plate_clone,
        &current_tcp_clone
    )
    .await;

    Ok(())
}

async fn chain_from_urdf_raw(
    robot_name: String,
    urdf: String,
) -> (Chain<f64>, Vec<String>, Vec<String>) {
    // create the temp directory to store the urdf file in
    let dir = match tempdir() {
        Ok(d) => d,
        Err(e) => {
            r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Failed to generate temporary urdf directory with: '{}'.",
                e
            );
            panic!() // OK to panic, makes no sense to continue without a urdf.
        }
    };

    // create the temporary urdf file
    let urdf_path = dir.path().join("temp_urdf.urdf");
    let mut file = match File::create(urdf_path.clone()) {
        Ok(f) => {
            r2r::log_info!(
                &format!("{robot_name}_control_ghost"),
                "Generated temporary urdf file at: {:?}",
                urdf_path
            );
            f
        }
        Err(e) => {
            r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Failed to generate temporary urdf file with: '{}'.",
                e
            );
            panic!() // OK to panic, makes no sense to continue without a urdf.
        }
    };

    // dump the raw urdf to the generated file
    match write!(file, "{}", urdf) {
        Ok(()) => (),
        Err(e) => {
            r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Failed to write to the temporary urdf file with: '{}'.",
                e
            );
            panic!() // OK to panic, makes no sense to continue without a urdf.
        }
    };

    let (c, j, l) = make_chain(
        &robot_name,
        match urdf_path.to_str() {
            Some(s) => s,
            None => {
                r2r::log_error!(
                    &format!("{robot_name}_control_ghost"),
                    "Failed to convert path to string slice."
                );
                panic!()
            }
        },
    )
    .await;

    drop(file);

    // once we have the chain, we don't need the urdf anymore
    match dir.close() {
        Ok(()) => (),
        Err(e) => {
            r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Failed to close and remove the temporary urdf directory with: '{}'.",
                e
            );
        }
    };

    (c, j, l)
}

// make the kinematic chain from the urdf file
async fn make_chain(robot_name: &str, urdf_path: &str) -> (Chain<f64>, Vec<String>, Vec<String>) {
    match k::Chain::<f64>::from_urdf_file(urdf_path) {
        Ok(c) => {
            r2r::log_info!(
                &format!("{robot_name}_control_ghost"),
                "Loading urdf file: '{:?}'.",
                urdf_path
            );
            (
                c.clone(),
                c.iter_joints()
                    .map(|j| j.name.clone())
                    .collect::<Vec<String>>(),
                c.iter_links()
                    .map(|l| l.name.clone())
                    .collect::<Vec<String>>(),
            )
        }
        Err(e) => {
            r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Failed to handle urdf with: '{}'.",
                e
            );
            panic!() // Still OK to panic, makes no sense to continue without a urdf.
        }
    }
}

// listen to the pose of the teaching marker so that the ghost knows where to go
// the ghost's chain will change whenever the actual chain changes
async fn pose_update(
    robot_name: String,
    current_chain: &Arc<Mutex<Chain<f64>>>,
    // current_face_plate_id: &Arc<Mutex<String>>,
    // current_tcp_id: &Arc<Mutex<String>>,
    current_faceplate_id: &Arc<Mutex<String>>,
    current_tcp_id: &Arc<Mutex<String>>,
    joint_state: &Arc<Mutex<JointState>>,
    global_transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        // let buffer = global_transform_buffer.lock().unwrap().clone();
        let mut new_joint_state = joint_state.lock().unwrap().clone();
        let current_chain_local = current_chain.lock().unwrap().clone();
        let current_faceplate_id_local = current_faceplate_id.lock().unwrap().clone();
        let current_tcp_id_local = current_tcp_id.lock().unwrap().clone();
        

        let transforms_all = global_transform_buffer.lock().unwrap().clone();
        println!("transforms: {:?}", transforms_all.keys());

        let teaching_marker_in_world = match lookup_transform_with_root(
            // &DEFAULT_BASEFRAME_ID,
            "a",
            // &format!("{robot_name}_ghost_marker").to_string(),
            "b",
            &DEFAULT_ROOT_FRAME_ID,
            &global_transform_buffer,
        ) {
            Some(transform) => transform,
            None => {
                r2r::log_error!(
                    &format!("{robot_name}_ghost_client"),
                    "Transform lookup has failed between frames {DEFAULT_BASEFRAME_ID} and {}.",
                    &format!("{robot_name}_ghost_marker")
                );
                timer.tick().await?;
                continue;
            }
        };

        let joint_state_local = joint_state.lock().unwrap().clone();
        match calculate_inverse_kinematics(
            robot_name.clone(),
            &current_chain_local,
            &current_faceplate_id_local,
            &current_tcp_id_local,
            &teaching_marker_in_world,
            &joint_state_local,
        )
        .await
        {
            Some(joints) => {
                new_joint_state.position = joints;
                *joint_state.lock().unwrap() = new_joint_state;
            }
            None => r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Calculating inverse kinematics failed."
            ),
        };
        timer.tick().await?;
    }
}

// get a joint position for the frame to go to
async fn calculate_inverse_kinematics(
    robot_name: String,
    new_chain: &Chain<f64>,
    face_plate_id: &str,
    tcp_id: &str,
    target_frame: &TransformStamped,
    act_joint_state: &JointState,
) -> Option<Vec<f64>> {
    match new_chain.find(&format!("{}-{}", face_plate_id, tcp_id)) {
        Some(ee_joint) => {
            // a chain can have branches, but a serial chain can't
            // so we use that instead to help the solver
            let arm = k::SerialChain::from_end(ee_joint);

            // since we have added a new joint, it is now a n + 1 DoF robot (if)
            let mut positions = act_joint_state.position.clone(); //.lock().unwrap().clone().position;
            positions.push(0.0); //(if we add anothet link)

            // the solver needs an initial joint position to be set.
            // check DoF so that this doesnt't fail when missmatch
            match arm.set_joint_positions(&positions) {
                Ok(()) => {
                    // will have to experiment with these solver parameters
                    // let solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 50);
                    let solver = k::JacobianIkSolver::default();

                    let target = Isometry3::from_parts(
                        Translation3::new(
                            target_frame.transform.translation.x as f64,
                            target_frame.transform.translation.y as f64,
                            target_frame.transform.translation.z as f64,
                        ),
                        UnitQuaternion::from_quaternion(Quaternion::new(
                            target_frame.transform.rotation.w as f64,
                            target_frame.transform.rotation.i as f64,
                            target_frame.transform.rotation.j as f64,
                            target_frame.transform.rotation.k as f64,
                        )),
                    );

                    // the last joint has to be rot type to be recognize, but we don't want it to roatate
                    let constraints = k::Constraints {
                        ignored_joint_names: vec![format!("{}-{}", face_plate_id, tcp_id)],
                        ..Default::default()
                    };

                    // solve, but with locking the last joint that we added
                    match solver.solve_with_constraints(&arm, &target, &constraints) {
                        Ok(()) => {
                            // get the solution and remove the (n + 1) - th '0.0' joint value
                            let mut j = arm.joint_positions();
                            match j.pop() {
                                Some(_) => Some(j),
                                None => {
                                    r2r::log_error!(
                                        &format!("{robot_name}_control_ghost"),
                                        "Failed to shrink joint dof to original size.",
                                    );
                                    None
                                }
                            }
                        }
                        Err(e) => {
                            r2r::log_error!(
                                &format!("{robot_name}_control_ghost"),
                                "Failed to solve with constraints with: '{}'.",
                                e
                            );
                            None
                        }
                    }
                }
                Err(e) => {
                    r2r::log_error!(
                        &format!("{robot_name}_control_ghost"),
                        "Failed to set joint positions for arm with: '{}'.",
                        e
                    );
                    None
                }
            }
        }
        None => None,
    }
}

//publish the ghost joint state
async fn ghost_publisher_callback(
    robot_name: String,
    publisher: r2r::Publisher<JointState>,
    mut timer: r2r::Timer,
    ghost_joint_state: &Arc<Mutex<JointState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let position = ghost_joint_state.lock().unwrap().clone().position;
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let updated_joint_state = JointState {
            header: Header {
                stamp: time_stamp.clone(),
                ..Default::default()
            },
            name: ghost_joint_state.lock().unwrap().clone().name,
            position,
            ..Default::default()
        };

        match publisher.publish(&updated_joint_state) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    &format!("{robot_name}_control_ghost"),
                    "Publisher failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

// // provide a service to reset the joint position of the ghost to the current actual robot state
// async fn reset_ghost_server(
//     mut service: impl Stream<Item = ServiceRequest<Trigger::Service>> + Unpin,
//     joint_state: &Arc<Mutex<JointState>>,
//     initial_joint_state: &JointState,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     loop {
//         match service.next().await {
//             Some(_) => {
//                 r2r::log_info!(&format!("{robot_name}_control_ghost"), "Got reset ghost request.");
//                 *joint_state.lock().unwrap() = initial_joint_state.clone();
//                 continue;
//             }
//             None => (),
//         }
//     }
// }

// the urdf only holds the joints and links of the robot that are always
// defined in a never-changing way. Sometimes, when the robot is expected
// to always use only one end effector and never change it, it could be reasonable
// to add a new 'fixed' joint and the end effector link to the urdf. In our
// use cases though, we would like to sometimes change tools, which changes the
// tool center point and thus the relationships to the face plate frame. Thus we
// always want to generate a new chain with the current configuration that we
// looked up from the tf. Also, an item's frame that is currently being held
// is also a reasonable tcp to be used when moving somewhere to leave the item.
async fn update_kinematic_chain(
    robot_name: String,
    global_transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
    chain: &Arc<Mutex<Chain<f64>>>,
    face_plate_id: &Arc<Mutex<String>>,
    tcp_id: &Arc<Mutex<String>>,
) -> Option<Chain<f64>> {
    let face_plate_id_local = &face_plate_id.lock().unwrap().clone();
    let tcp_id_local = &tcp_id.lock().unwrap().clone();

    let tcp_in_face_plate = match lookup_transform_with_root(
        face_plate_id_local,
        tcp_id_local,
        &DEFAULT_ROOT_FRAME_ID,
        &global_transform_buffer,
    ) {
        Some(transform) => transform,
        None => {
            r2r::log_error!(
                &format!("{robot_name}_ghost_client"),
                "Transform lookup has failed between frames {face_plate_id_local} and {tcp_id_local}."
            );
            return None;
        }
    };

    let face_plate_to_tcp_joint: k::Node<f64> = k::NodeBuilder::<f64>::new()
        .name(&format!("{}-{}", face_plate_id_local, tcp_id_local))
        .translation(Translation3::new(
            tcp_in_face_plate.transform.translation.x as f64,
            tcp_in_face_plate.transform.translation.y as f64,
            tcp_in_face_plate.transform.translation.z as f64,
        ))
        .rotation(UnitQuaternion::from_quaternion(Quaternion::new(
            tcp_in_face_plate.transform.rotation.w as f64,
            tcp_in_face_plate.transform.rotation.i as f64,
            tcp_in_face_plate.transform.rotation.j as f64,
            tcp_in_face_plate.transform.rotation.k as f64,
        )))
        // have to make a rot joint, a fixed one is not recognized in DoF
        .joint_type(k::JointType::Rotational {
            axis: k::Vector3::y_axis(),
        })
        .finalize()
        .into();

    // specify the tcp link
    let tcp_link = k::link::LinkBuilder::new().name(tcp_id_local).finalize();
    face_plate_to_tcp_joint.set_link(Some(tcp_link));

    let old_chain = chain.lock().unwrap().clone();

    // get the last joint in the chain and hope to get the right one xD
    match old_chain
        .iter_joints()
        .map(|j| j.name.clone())
        .collect::<Vec<String>>()
        .last()
    {
        // fetch the node that is specified by the last joint
        Some(parent) => match old_chain.find(parent) {
            Some(parent_node) => {
                // specify the parent of the newly made face_plate-tcp joint
                face_plate_to_tcp_joint.set_parent(parent_node);

                // get all the nodes in the chain
                let mut new_chain_nodes: Vec<k::Node<f64>> =
                    old_chain.iter().map(|x| x.clone()).collect();

                // add the new joint and generate the new chain
                new_chain_nodes.push(face_plate_to_tcp_joint);
                let new_chain = Chain::from_nodes(new_chain_nodes);
                *chain.lock().unwrap() = new_chain.clone();
                Some(new_chain)
            }
            None => {
                r2r::log_error!(
                    &format!("{robot_name}_control_ghost"),
                    "Failed to set parent node."
                );
                None
            }
        },
        None => {
            r2r::log_error!(
                &format!("{robot_name}_control_ghost"),
                "Failed to find parent node in the chain."
            );
            None
        }
    }
}
