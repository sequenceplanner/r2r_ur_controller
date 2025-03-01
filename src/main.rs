use micro_sp::*;
use r2r_transforms::*;
use r2r_ur_controller::*;
use std::path::PathBuf;

use std::error::Error;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

// This is a test, use the following as an example in your code
pub static NODE_ID: &'static str = "ur_controller";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let robot_name = "ur1";

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");

    let mut path = PathBuf::from(&manifest_dir);
    path.pop();
    path.pop();

    path.push("src/ur_description/urdf/ur.urdf.xacro");

    println!("{:?}", path);

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

    let state = generate_robot_interface_state(&robot_name);
    let (tx, rx) = mpsc::channel(50);
    tokio::spawn(state_manager(rx, state));

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");
    let templates: tera::Tera = {
        let tera = match tera::Tera::new(&format!("{}/templates/*.script", manifest_dir)) {
            Ok(t) => {
                r2r::log_warn!(NODE_ID, "Searching for Tera templates, wait...",);
                t
            }
            Err(e) => {
                r2r::log_error!(NODE_ID, "UR Script template parsing error(s): {}", e);
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
        r2r::log_error!(NODE_ID, "Couldn't find any Tera templates.");
    } else {
        r2r::log_info!(NODE_ID, "Found templates.");
    }
    // for template in &template_names {
    //     r2r::log_info!(NODE_ID, "Found template: {:?}", template);
    // }

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let transform_buffer = RosSpaceTreeServer::new("buffer", &arc_node_clone);
    // let transform_buffer = SpaceTreeServer::new("buffer");

    let mut base_in_world = TransformStamped::default();
    base_in_world.active = false;
    base_in_world.child_frame_id = "base".to_string();
    base_in_world.parent_frame_id = "world".to_string();

    let mut a_in_world = TransformStamped::default();
    a_in_world.active = false;
    a_in_world.child_frame_id = "a".to_string();
    a_in_world.parent_frame_id = "world".to_string();
    a_in_world.transform.translation.x = 0.6;

    let mut b_in_world = TransformStamped::default();
    b_in_world.active = false;
    b_in_world.child_frame_id = "b".to_string();
    b_in_world.parent_frame_id = "world".to_string();
    b_in_world.transform.translation.y = 0.6;

    let mut ghost_base_link_in_base = TransformStamped::default();
    ghost_base_link_in_base.active = false;
    ghost_base_link_in_base.child_frame_id = "ghost_base_link".to_string();
    ghost_base_link_in_base.parent_frame_id = "world".to_string();

    transform_buffer.insert_transform("base", base_in_world);
    transform_buffer.insert_transform("ghost_base_link", ghost_base_link_in_base);
    transform_buffer.insert_transform("a", a_in_world);
    transform_buffer.insert_transform("b", b_in_world);
    transform_buffer.apply_changes();

    // println!("local: {:?}", transform_buffer.get_local_transform_names());
    // println!("global: {:?}", transform_buffer.get_global_transform_names());

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    tokio::task::spawn(async move {
        joint_subscriber(
            &robot_name,
            arc_node_clone,
            tx_clone,
        )
        .await
        .unwrap()
    });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    let global_buffer = transform_buffer.global_buffer.clone();
    tokio::task::spawn(async move {
        action_client(
            &robot_name,
            arc_node_clone,
            tx_clone,
            &global_buffer,
            &templates,
        )
        .await
        .unwrap()
    });

    let global_buffer = transform_buffer.global_buffer.clone();
    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    let urdf_clone = urdf.clone();
    tokio::task::spawn(async move {
        control_ghost(
            robot_name.to_string(),
            urdf_clone,
            arc_node_clone,
            tx_clone,
            &global_buffer,
        )
        .await
        .unwrap()
    });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    tokio::task::spawn(async move {
        dashboard_client(&robot_name, arc_node_clone, tx_clone)
            .await
            .unwrap()
    });

    tokio::task::spawn(async move {
        ur_script_driver(Some("172.17.0.1".to_string()))
            .await
            .unwrap()
    });

    tokio::task::spawn(async move { robot_state_publisher(&urdf, "").await.unwrap() });

    let mut ghost_params = URDFParameters::default();
    ghost_params.tf_prefix = "ghost_".to_string();

    ghost_params.description_file = urdf_path; //format!("{}/src/description/urdf/ur.urdf.xacro", manifest_dir);
    let ghost_urdf = match convert_xacro_to_urdf(ghost_params) {
        Some(urdf) => urdf,
        None => panic!("Failed to parse urdf."),
    };

    tokio::task::spawn(async move { robot_state_publisher(&ghost_urdf, "ghost").await.unwrap() });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let handle = std::thread::spawn(move || loop {
        arc_node_clone
            .lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}
