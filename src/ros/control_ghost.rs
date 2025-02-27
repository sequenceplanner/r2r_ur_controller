use futures::{Stream, StreamExt};
use k::nalgebra::Quaternion;
use k::prelude::InverseKinematicsSolver;
use k::{Chain, Node};
use k::{Isometry3, Translation3, UnitQuaternion, Vector3};
use micro_sp::ToSPValue;
use micro_sp::*;
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::ur_script_msgs::srv::DashboardCommand as DBCommand;
use r2r::QosProfile;
use r2r_teaching_markers::TeachingMarkerServer;
use r2r_transforms::*;
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

pub async fn control_ghost(
    robot_name: &str,
    tcp_id: &str,
    urdf: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>,
    transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>, // has to be global
) -> Result<(), Box<dyn std::error::Error>> {
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

    r2r::log_info!(
        &format!("{robot_name}_control_ghost"),
        "Starting control ghost."
    );

    // Make a manipulatable kinematic chain using the urdf
    let (chain, joints, links) = chain_from_urdf_raw(robot_name, urdf).await;

    let initial_joint_position_value = loop {
        let (response_tx, response_rx) = oneshot::channel();
        state_mgmt
            .send(StateManagement::Get((
                format!("{robot_name}_joint_states"),
                response_tx,
            )))
            .await?;
        let joint_state = response_rx.await?;
        if joint_state != SPValue::UNKNOWN {
            break match joint_state {
                SPValue::Array(SPValueType::Float64, joints) => joints
                    .iter()
                    .map(|val| match val {
                        SPValue::Float64(value) => value.into_inner(),
                        _ => panic!("Joint state has to ba array of f64."),
                    })
                    .collect::<Vec<f64>>(),
                _ => panic!("Joint state has to ba array of f64."),
            };
        }
        timer.tick().await?;
    };

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
        tcp_id.to_string(),
        None,
        arc_node_clone,
    );

    let static_tf_listener = arc_node
        .lock()
        .unwrap()
        .subscribe::<TFMessage>("tf_static", QosProfile::volatile(QosProfile::default()))
        .expect("Failed to initialize static_tf_listener.");

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

    // spawn a tokio task to listen to incomming teaching marker poses
    let joint_state_clone = joint_states.clone();
    let current_chain_clone = chain.clone();
    // let current_tcp_clone = current_tcp.clone();
    tokio::task::spawn(async move {
        match teaching_marker_callback(
            robot_name.to_string(),
            static_tf_listener,
            &current_chain_clone,
            &current_tcp_clone,
            &joint_state_clone,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Teaching mode subscriber failed with {}.", e),
        };
    });

    // let robot_name_string_clone = robot_name_string.clone();
    // let robot_name_string_clone_2 = robot_name_string.clone();
    // tokio::task::spawn(async move {
    //     match ghost_publisher_callback(robot_name_string_clone, ghost_state_publisher, timer, &joint_states_clone).await
    //     {
    //         Ok(()) => (),
    //         Err(e) => r2r::log_error!(&format!("{robot_name_string_clone_2}_ghost_marker"), "Joint state publisher failed with: '{}'.", e),
    //     };
    // });

    // loop {
    //     let (response_tx, response_rx) = oneshot::channel();

    //     let tcp_in_faceplace = match lookup_transform_with_root(
    //         &DEFAULT_FACEPLATE_ID, //use state instead and define this during launch
    //         &tcp_id,
    //         &DEFAULT_ROOT_FRAME_ID,
    //         transform_buffer,
    //     ) {
    //         Some(transform) => transform,
    //         None => {
    //             r2r::log_error!(
    //                 &format!("{robot_name}_ghost_marker"),
    //                 "Transform lookup has failed between frames {} and {}.", DEFAULT_FACEPLATE_ID, &tcp_id);
    //             TransformStamped::default()
    //         }
    //     };

    // // transform_buffer.lock().unwrap().clone()
    // state_mgmt
    //     .send(StateManagement::GetState(response_tx))
    //     .await?;
    // let state = response_rx.await?;

    // timer.tick().await?;
    // }
    Ok(())
}

async fn chain_from_urdf_raw(
    robot_name: &str,
    urdf: &str,
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
        robot_name,
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

// actually make the kinematic chain from the urdf file (supplied or generated)
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
async fn teaching_marker_callback(
    robot_name: String,
    mut subscriber: impl Stream<Item = TransformStamped> + Unpin,
    current_chain: &Arc<Mutex<Chain<f64>>>,
    // current_face_plate_id: &Arc<Mutex<String>>,
    // current_tcp_id: &Arc<Mutex<String>>,
    current_tcp_id: String,
    joint_state: &Arc<Mutex<JointState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(msg) => {
                let mut new_joint_state = joint_state.lock().unwrap().clone();
                let current_chain_local = current_chain.lock().unwrap().clone();
                // let current_face_plate_id_local = current_face_plate_id.lock().unwrap().clone();
                let current_tcp_id_local = current_tcp_id.clone();
                let current_face_plate_id_local = DEFAULT_FACEPLATE_ID;
                let joint_state_local = joint_state.lock().unwrap().clone();
                match (current_face_plate_id_local != "unknown")
                    & (current_tcp_id_local != "unknown")
                {
                    true => {
                        match calculate_inverse_kinematics(
                            robot_name.clone(),
                            &current_chain_local,
                            &current_face_plate_id_local,
                            &current_tcp_id_local,
                            &msg.,
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
                    }
                    false => r2r::log_error!(
                        &format!("{robot_name}_control_ghost"),
                        "What is unknown?: {:?}, {:?}",
                        current_face_plate_id_local,
                        current_tcp_id_local
                    ),
                }
            }
            None => {
                r2r::log_error!(
                    &format!("{robot_name}_control_ghost"),
                    "Subscriber did not get the message?"
                );
            }
        }
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

            // since we have added a new joint, it is now a n + 1 DoF robot
            let mut positions = act_joint_state.position.clone(); //.lock().unwrap().clone().position;
            positions.push(0.0);

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
                                        "Failed to shring joint dof to original size.",
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
