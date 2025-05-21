use std::error::Error;
use tokio::{
    sync::{mpsc, oneshot},
    time::{interval, Duration},
};

use micro_sp::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Logs from extern crates to stdout
    let mut interval = interval(Duration::from_millis(100));
    initialize_env_logger();

    // let state = generate_robot_interface_state("r1");
    let (tx, rx) = mpsc::channel(50);

    tokio::task::spawn(async move {
        match redis_state_manager(rx, State::new()).await {
            Ok(()) => (),
            Err(e) => log::error!(target: &&format!("r2r_ur_controller"), "{}", e),
        };
    });

    tokio::task::spawn(async move {
        match trigger(tx).await {
            Ok(()) => (),
            Err(e) => log::error!(target: &&format!("r2r_ur_controller"), "{}", e),
        };
    });


    loop {
        interval.tick().await;
    }
}

async fn trigger(
    command_sender: mpsc::Sender<StateManagement>,
) -> Result<(), Box<dyn Error>> {
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    log::info!(target: "dummy_trigger", "Trying to move the robot");
    // let goal = "var:robot_mounted_estimated == suction_tool";

    let (response_tx, response_rx) = oneshot::channel();
    command_sender
        .send(StateManagement::GetState(response_tx))
        .await?; // TODO: maybe we can just ask for values from the guard
    let state = response_rx.await?;

    let new_state = state
        // Optional to test what happens when... (look in the Emulation msg for details)
        .update("r1_velocity", 0.3.to_spvalue())
        .update("r1_accelleration", 0.3.to_spvalue())
        .update("r1_tcp_id", "suction_cup_1".to_spvalue())
        .update("r1_faceplate_id", "tool0".to_spvalue())
        .update("r1_baseframe_id", "base_link".to_spvalue())
        .update("r1_goal_feature_id", "above_buffer_1".to_spvalue())
        // .update("r1_use_joint_positions", true.to_spvalue())
        // .update("r1_joint_positions", vec!(0.0, 0.0, 0.0, -1.5707, 0.0, 0.0).to_spvalue())
        .update("r1_command_type", "move_j".to_spvalue());

    let modified_state = state.get_diff_partial_state(&new_state);
    command_sender
        .send(StateManagement::SetPartialState(modified_state))
        .await?;

    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    let new_state = state
        .update("r1_request_trigger", true.to_spvalue())
        .update("r1_request_state", "initial".to_spvalue());

    let modified_state = state.get_diff_partial_state(&new_state);
    command_sender
        .send(StateManagement::SetPartialState(modified_state))
        .await?;



    // r2r::log_warn!(NODE_ID, "All tests are finished. Generating report...");

    // TODO
    // Measure operation and plan execution times, and measure total failure rates...
    // Print out plan done or plan failed when done or failed...

    Ok(())
}
