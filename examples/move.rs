use std::{error::Error, sync::Arc};
use tokio::time::{interval, Duration};

use micro_sp::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Logs from extern crates to stdout
    let mut interval = interval(Duration::from_millis(500));
    initialize_env_logger();

    // let (tx, rx) = mpsc::channel(50);

    let connection_manager = ConnectionManager::new().await;
    // StateManager::set_state(&mut connection_manager.get_connection().await, state).await;
    let con_arc = Arc::new(connection_manager);

    tokio::task::spawn(async move {
        match trigger(con_arc).await {
            Ok(()) => (),
            Err(e) => log::error!(target: &&format!("r2r_ur_controller"), "{}", e),
        };
    });

    loop {
        interval.tick().await;
    }
}

async fn trigger(connection_manager: Arc<ConnectionManager>) -> Result<(), Box<dyn Error>> {
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    log::info!(target: "dummy_trigger", "Trying to move the robot");
    // let goal = "var:robot_mounted_estimated == suction_tool";
    let mut con = connection_manager.get_connection().await;
    let state = match StateManager::get_full_state(&mut con).await {
        Some(s) => s,
        None => panic!("no state"),
    };

    let new_state = state
        // Optional to test what happens when... (look in the Emulation msg for details)
        .update("r1_velocity", 0.02.to_spvalue())
        .update("r1_accelleration", 0.1.to_spvalue())
        .update("r1_request_trigger", false.to_spvalue())
        .update("r1_request_state", "initial".to_spvalue())
        .update("r1_force_threshold", 20.0.to_spvalue())
        // .update("r1_tcp_id", "suction_cup_1".to_spvalue())
        // .update("r1_faceplate_id", "tool0".to_spvalue())
        // .update("r1_baseframe_id", "base_link".to_spvalue())
        // .update("r1_goal_feature_id", "above_buffer_1".to_spvalue())
        .update("r1_command_type", "safe_move_l_relative".to_spvalue())
        .update("r1_relative_pose", "p[0.0, 0.0, 0.2, 0.0, 0.0, 0.0]".to_spvalue());

    let modified_state = state.get_diff_partial_state(&new_state);
    StateManager::set_state(&mut con, &modified_state).await;

    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    let new_state = state
        .update("r1_request_trigger", true.to_spvalue())
        .update("r1_request_state", "initial".to_spvalue());

    let modified_state = state.get_diff_partial_state(&new_state);
    StateManager::set_state(&mut con, &modified_state).await;

    Ok(())
}
