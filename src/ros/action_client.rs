use std::sync::{Arc, Mutex};

use futures::Future;
use micro_sp::{State, ToSPValue};
use r2r::sp_ariac_msgs::action::RobotCommand;
use r2r::ActionServerGoal;
use r2r::Error;

use crate::*;

pub async fn ur_action_client(
    action_topic: &str,
    request_trigger_var: &str,
    request_state_var: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    shared_state: &Arc<Mutex<State>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let client = arc_node
        .lock()
        .unwrap()
        .create_action_client::<RobotCommand::Action>(action_topic)?;
    let waiting_for_server = arc_node.lock().unwrap().is_available(&client)?;
    let timer = arc_node
        .lock()
        .unwrap()
        .create_wall_timer(std::time::Duration::from_millis(
            UR_ACTION_SERVER_TICKER_RATE,
        ))?;

    r2r::log_warn!(NODE_ID, "Waiting for the robot action server...");
    wait_for_server.await?;
    r2r::log_info!(NODE_ID, "Robot action server available.");

    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let request_trigger =
            match shared_state_local.get_value(request_trigger_var) {
                micro_sp::SPValue::Bool(value) => value,
                _ => {
                    r2r::log_error!(
                        NODE_ID,
                        "Couldn't get {} from the shared state.", request_trigger_var
                    );
                    false
                }
            };
        let request_state =
            match shared_state_local.get_value(request_state_var) {
                micro_sp::SPValue::String(value) => value,
                _ => {
                    r2r::log_error!(
                        NODE_ID,
                        "Couldn't get {} from the shared state.", request_state_var
                    );
                    "unknown".to_string()
                }
            };
        }
    }


    // let mut goal_handle_status = None;

    // let goal = RobotCommand::Goal {
    //     command: "asdf".to_string(),
    // };

    // // let competition_state = competition_state.lock().unwrap().clone();
    // // match competition_state {
    //     // CompetitionState::Started => {
    //         let (goal_handle, result, feedback) = match robot_action_client.send_goal_request(goal)
    //         {
    //             Ok(x) => match x.await {
    //                 Ok(y) => y,
    //                 Err(e) => {
    //                     r2r::log_info!(NODE_ID, "Could not send goal request.");
    //                     return Err(Box::new(e));
    //                 }
    //             },
    //             Err(e) => {
    //                 r2r::log_info!(NODE_ID, "Did not get goal.");
    //                 return Err(Box::new(e));
    //             }
    //         };

    //         goal_handle_status = match goal_handle.get_status() {
    //             Ok(status) => match status {
    //                 r2r::GoalStatus::Accepted => {
    //                     // let shared_state_local =
    //                     //     shared_state_local.update("ur_action_state", "executing".to_spvalue());
    //                     // *shared_state.lock().unwrap() = shared_state_local;
    //                     Some("accepted")
    //                 }
    //                 _ => {
    //                     // let shared_state_local =
    //                     //     shared_state_local.update("ur_action_state", "failed".to_spvalue());
    //                     // *shared_state.lock().unwrap() = shared_state_local;
    //                     None
    //                 }
    //             },
    //             Err(_) => None,
    //         };

    //         match result.await {
    //             Ok((status, msg)) => match status {
    //                 r2r::GoalStatus::Aborted => {
    //                     r2r::log_info!(NODE_ID, "Goal succesfully aborted with: {:?}", msg);
    //                     // let _ = g.publish_feedback(URCommand::Feedback {
    //                     //     current_state: "Goal succesfully aborted.".into(),
    //                     // });
    //                     // Ok(())
    //                 }
    //                 _ => {
    //                     r2r::log_info!(
    //                         NODE_ID,
    //                         "Executing the robot action communication succeeded."
    //                     );
    //                     // let shared_state_local =
    //                     //     shared_state_local.update("ur_action_state", "succeeded".to_spvalue());
    //                     // *shared_state.lock().unwrap() = shared_state_local;
    //                     // let _ = g.publish_feedback(URCommand::Feedback {
    //                     //     current_state: "Executing the Simple Robot Simulator action succeeded.".into(),
    //                     // });
    //                     // Ok(())
    //                 }
    //             },
    //             Err(e) => {}
    //         }
    //     // }
    //     // _ => (),
    // // }
    // Ok(())
}


// pub async fn hl_floor_robot_action_client_ticker(
//     robot_action_client: &r2r::ActionClient<RobotCommand::Action>,
//     wait_for_server: impl Future<Output = Result<(), Error>>,
//     // competition_state: &Arc<Mutex<State>>,
//     mut timer: r2r::Timer
// ) -> Result<(), Box<dyn std::error::Error>> {
    
// }