use micro_sp::*;
use r2r_ur_controller::ros::robot_state_to_redis::robot_state_to_redis;
use r2r_ur_controller::*;
use std::path::PathBuf;

use std::error::Error;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

// This is a test, use the following as an example in your code
pub static NODE_ID: &'static str = "r2r_ur_controller";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    let robot_id = match std::env::var("ROBOT_ID") {
        Ok(id) => id,
        Err(e) => {
            log::error!(target: &&format!("r2r_ur_controller"), "Failed to read ROBOT_ID environment variable: {}", e);
            log::error!(target: &&format!("r2r_ur_controller"), "Setting ROBOT_ID to robot.");
            "robot".to_string()
        }
    };
    let urdf_dir = std::env::var("URDF_DIR").expect("URDF_DIR is not set");
    let templates_dir = std::env::var("TEMPLATES_DIR").expect("TEMPLATES_DIR is not set");
    let override_host = match std::env::var("OVERRIDE_HOST") {
        Ok(val_str) => match val_str.to_lowercase().parse::<bool>() {
            Ok(b_val) => b_val,
            Err(e) => {
                log::error!(target: &&format!("r2r_ur_controller"), "Failed to parse OVERRIDE_HOST value '{}' as boolean: {}", val_str, e);
                log::error!(target: &&format!("r2r_ur_controller"), "Seeting OVERRIDE_HOST to false.");
                false
            }
        },
        Err(e) => {
            log::error!(target: &&format!("r2r_ur_controller"), "Failed to read OVERRIDE_HOST environment variable: {}", e);
            log::error!(target: &&format!("r2r_ur_controller"), "Seeting OVERRIDE_HOST to false.");
            false
        }
    };
    let override_host_addess =
        std::env::var("OVERRIDE_HOST_ADDRESS").expect("OVERRIDE_HOST_ADDRESS is not set");
    let ur_address = std::env::var("UR_ADDRESS").expect("UR_ADDRESS is not set");

    let mut path = PathBuf::from(&urdf_dir);
    path.push("ur.urdf.xacro");

    let urdf_path = path.to_string_lossy().to_string();

    let mut params = URDFParameters::default();

    params.description_file = urdf_path.clone(); //format!("{}/src/description/urdf/ur.urdf.xacro", manifest_dir);
    let urdf = match convert_xacro_to_urdf(params) {
        Some(urdf) => urdf,
        None => panic!("Failed to parse urdf."),
    };

    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, NODE_ID, "")?;
    let arc_node = Arc::new(Mutex::new(node));

    let state = generate_robot_interface_state(&robot_id);
    let (tx, rx) = mpsc::channel(500);

    tokio::task::spawn(async move {
        match redis_state_manager(rx, state).await {
            Ok(()) => (),
            Err(e) => log::error!(target: &&format!("r2r_ur_controller"), "{}", e),
        };
    });

    let templates: tera::Tera = {
        let tera = match tera::Tera::new(&format!("{}/*.script", templates_dir)) {
            Ok(t) => {
                log::warn!(target: &&format!("r2r_ur_controller"), "Searching for Tera templates, wait...",);
                t
            }
            Err(e) => {
                log::error!(target: &&format!("r2r_ur_controller"), "UR Script template parsing error(s): {}", e);
                ::std::process::exit(1);
            }
        };
        tera
    };

    let template_names = templates
        .get_template_names()
        .map(|x| x.to_string())
        .collect::<Vec<String>>();
    if template_names.len() == 0 {
        log::error!(target: &&format!("r2r_ur_controller"), "Couldn't find any Tera templates.");
    } else {
        log::info!(target: &&format!("r2r_ur_controller"), "Found templates.");
    }

    // let base_in_world = SPTransformStamped {
    //     active_transform: false,
    //     child_frame_id: "base".to_string(),
    //     parent_frame_id: "world".to_string(),
    //     enable_transform: true,
    //     time_stamp: SystemTime::now(),
    //     transform: SPTransform::default(),
    //     metadata: MapOrUnknown::UNKNOWN,
    // };

    // base_in_world.active = false;
    // base_in_world.child_frame_id = "base".to_string();
    // base_in_world.parent_frame_id = "world".to_string();

    // let mut a_in_world = TransformStamped::default();
    // a_in_world.active = false;
    // a_in_world.child_frame_id = "a".to_string();
    // a_in_world.parent_frame_id = "world".to_string();
    // a_in_world.transform.translation.x = 0.6;

    // let mut b_in_world = TransformStamped::default();
    // b_in_world.active = false;
    // b_in_world.child_frame_id = "b".to_string();
    // b_in_world.parent_frame_id = "world".to_string();
    // b_in_world.transform.translation.y = 0.6;

    // let mut ghost_base_link_in_base = TransformStamped::default();
    // ghost_base_link_in_base.active = false;
    // ghost_base_link_in_base.child_frame_id = "ghost_base_link".to_string();
    // ghost_base_link_in_base.parent_frame_id = "base".to_string();

    // tx.send(StateManagement::InsertTransform((
    //     base_in_world.child_frame_id.clone(),
    //     base_in_world,
    // )))
    // .await
    // .expect("failed");

    // println!("local: {:?}", transform_buffer.get_local_transform_names());
    // println!("global: {:?}", transform_buffer.get_global_transform_names());


    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    let robot_id_clone = robot_id.clone();
    tokio::task::spawn(async move {
        match action_client(&robot_id_clone, arc_node_clone, tx_clone, &templates).await {
            Ok(()) => (),
            Err(e) => {
                log::error!(target: &&format!("main robot runner"), "failed with: {}", e)
            }
        }
    });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    let robot_id_clone = robot_id.clone();
    tokio::task::spawn(async move {
        joint_subscriber(&robot_id_clone, arc_node_clone, tx_clone)
            .await
            .unwrap()
    });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    let robot_id_clone = robot_id.clone();
    tokio::task::spawn(async move {
        robot_state_to_redis(&robot_id_clone, arc_node_clone, tx_clone)
            .await
            .unwrap()
    });


    // tokio::task::spawn(async move {
    //     ur_script_driver(
    //         Some(ur_address),
    //         if override_host {
    //             Some(override_host_addess)
    //         } else {
    //             None
    //         },
    //     )
    //     .await
    //     .unwrap()
    // });

    // tokio::task::spawn(async move { robot_state_publisher(&urdf, "").await.unwrap() });

    // let mut ghost_params = URDFParameters::default();
    // ghost_params.tf_prefix = "ghost_".to_string();

    // ghost_params.description_file = urdf_path; //format!("{}/src/description/urdf/ur.urdf.xacro", manifest_dir);
    // let ghost_urdf = match convert_xacro_to_urdf(ghost_params) {
    //     Some(urdf) => urdf,
    //     None => panic!("Failed to parse urdf."),
    // };

    // tokio::task::spawn(async move { robot_state_publisher(&ghost_urdf, "ghost_").await.unwrap() });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let handle = std::thread::spawn(move || loop {
        arc_node_clone
            .lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}
