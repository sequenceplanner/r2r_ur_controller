use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use futures::Future;
use micro_sp::{ActionRequestState, SPValueType, State, ToSPValue};
use r2r::ur_script_msgs::action::ExecuteScript;
use r2r::ActionServerGoal;
use r2r::Error;
use r2r_transforms::TransformStamped;

use crate::core::structs::CommandType;
use crate::*;

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 500;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

pub async fn action_client(
    robot_name: &str,
    action_topic: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    shared_state: &Arc<Mutex<State>>,
    transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    let client = arc_node
        .lock()
        .unwrap()
        .create_action_client::<ExecuteScript::Action>(&format!("{robot_name}_ur_script"))?;
    let waiting_for_server = r2r::Node::is_available(&client)?;
    let timer = arc_node
        .lock()
        .unwrap()
        .create_wall_timer(std::time::Duration::from_millis(
            UR_ACTION_SERVER_TICKER_RATE,
        ))?;

    r2r::log_warn!(
        &format!("{robot_name}_action_client"),
        "Waiting for the {robot_name} control action server..."
    );

    waiting_for_server.await?;
    r2r::log_info!(
        &format!("{robot_name}_action_client"),
        "Robot {robot_name} control action server available."
    );

    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let request_trigger =
            match shared_state_local.get_value(&format!("{robot_name}_request_trigger")) {
                micro_sp::SPValue::Bool(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_request_trigger")
                    );
                    false
                }
            };
        let request_state =
            match shared_state_local.get_value(&format!("{robot_name}_request_state")) {
                micro_sp::SPValue::String(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_request_state")
                    );
                    "unknown".to_string()
                }
            };

        let command_type = match shared_state_local.get_value(&format!("{robot_name}_command_type"))
        {
            micro_sp::SPValue::String(value) => value,
            _ => {
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "Couldn't get {} from the shared state.",
                    &format!("{robot_name}_command_type")
                );
                CommandType::UNKNOWN.to_string()
            }
        };

        let accelleration =
            match shared_state_local.get_value(&format!("{robot_name}_accelleration")) {
                micro_sp::SPValue::Float64(value) => value.into_inner(),
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_accelleration")
                    );
                    0.0
                }
            };

        let velocity = match shared_state_local.get_value(&format!("{robot_name}_velocity")) {
            micro_sp::SPValue::Float64(value) => value.into_inner(),
            _ => {
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "Couldn't get {} from the shared state.",
                    &format!("{robot_name}_velocity")
                );
                0.0
            }
        };

        let global_acceleration_scaling = match shared_state_local
            .get_value(&format!("{robot_name}_global_acceleration_scaling"))
        {
            micro_sp::SPValue::Float64(value) => value.into_inner(),
            _ => {
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "Couldn't get {} from the shared state.",
                    &format!("{robot_name}_global_acceleration_scaling")
                );
                0.0
            }
        };

        let global_velocity_scaling =
            match shared_state_local.get_value(&format!("{robot_name}_global_velocity_scaling")) {
                micro_sp::SPValue::Float64(value) => value.into_inner(),
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_global_velocity_scaling")
                    );
                    0.0
                }
            };

        let use_execution_time =
            match shared_state_local.get_value(&format!("{robot_name}_use_execution_time")) {
                micro_sp::SPValue::Bool(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_use_execution_time")
                    );
                    false
                }
            };

        let execution_time =
            match shared_state_local.get_value(&format!("{robot_name}_execution_time")) {
                micro_sp::SPValue::Float64(value) => value.into_inner(),
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_execution_time")
                    );
                    0.0
                }
            };

        let use_blend_radius =
            match shared_state_local.get_value(&format!("{robot_name}_use_blend_radius")) {
                micro_sp::SPValue::Bool(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_use_blend_radius")
                    );
                    false
                }
            };

        let blend_radius = match shared_state_local.get_value(&format!("{robot_name}_blend_radius"))
        {
            micro_sp::SPValue::Float64(value) => value.into_inner(),
            _ => {
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "Couldn't get {} from the shared state.",
                    &format!("{robot_name}_blend_radius")
                );
                0.0
            }
        };

        let use_joint_positions =
            match shared_state_local.get_value(&format!("{robot_name}_use_joint_positions")) {
                micro_sp::SPValue::Bool(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_use_joint_positions")
                    );
                    false
                }
            };

        let joint_positions =
            match shared_state_local.get_value(&format!("{robot_name}_joint_positions")) {
                micro_sp::SPValue::Array(SPValueType::Float64, values) => values
                    .iter()
                    .enumerate()
                    .map(|(i, val)| match val {
                        micro_sp::SPValue::Float64(ordered_float) => ordered_float.into_inner(),
                        _ => SAFE_HOME_JOINT_STATE[i],
                    })
                    .collect(),
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_joint_positions")
                    );
                    SAFE_HOME_JOINT_STATE.to_vec()
                }
            };

        let use_preferred_joint_config = match shared_state_local
            .get_value(&format!("{robot_name}_use_preferred_joint_config"))
        {
            micro_sp::SPValue::Bool(value) => value,
            _ => {
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "Couldn't get {} from the shared state.",
                    &format!("{robot_name}_use_preferred_joint_config")
                );
                false
            }
        };

        let preferred_joint_config =
            match shared_state_local.get_value(&format!("{robot_name}_preferred_joint_config")) {
                micro_sp::SPValue::Array(SPValueType::Float64, values) => values
                    .iter()
                    .enumerate()
                    .map(|(i, val)| match val {
                        micro_sp::SPValue::Float64(ordered_float) => ordered_float.into_inner(),
                        _ => SAFE_HOME_JOINT_STATE[i],
                    })
                    .collect(),
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_action_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_preferred_joint_config")
                    );
                    SAFE_HOME_JOINT_STATE.to_vec()
                }
            };

        // let... add other robot specific things like speed, acc, position, etc.

        if request_trigger {
            if request_state == ActionRequestState::Initial.to_string() {
                // then prepare the URScript and just sent to the execute ur_script_action

                let robot_command = RobotCommand {
                    command_type,
                    acceleration: accelleration,
                    velocity,
                    global_acceleration_scaling,
                    global_velocity_scaling,
                    use_execution_time,
                    execution_time,
                    use_blend_radius,
                    blend_radius,
                    use_joint_positions,
                    joint_positions,
                    use_preferred_joint_config,
                    preferred_joint_config,
                    use_payload: todo!(),
                    payload: todo!(),
                    baseframe_id: todo!(),
                    faceplate_id: todo!(),
                    goal_feature_id: todo!(),
                    tcp_id: todo!(),
                    target_in_base: todo!(),
                    set_tcp: todo!(),
                    tcp_in_faceplate: todo!(),
                    // other things from the message
                };

                let goal = match generate_script(robot_command, templates).await {
                    Some(script) => ExecuteScript::Goal { script },
                    None => return Ok(ActionResult::Bool(false)), // RETURN ERROR SOMEHOW: Err(std::error::Error::default())
                };

                let mut goal_handle_status = None;

                let (goal_handle, result, mut feedback) = match urc_client.send_goal_request(goal) {
                    Ok(x) => match x.await {
                        Ok(y) => y,
                        Err(e) => {
                            r2r::log_info!(NODE_ID, "Could not send goal request.");
                            return Err(Box::new(e));
                        }
                    },
                    Err(e) => {
                        r2r::log_info!(NODE_ID, "Did not get goal.");
                        return Err(Box::new(e));
                    }
                };
            }
        }
    }

    Ok(())
}

//     // if lock_agv_three_tray_request_trigger {
//     //     if lock_agv_three_tray_request_state == ServiceRequestState::Initial.to_string() {
//     //         r2r::log_info!(NODE_ID, "Locking AGV 3's tray.");
//     //         let request = TriggerMsg::Request {};
//     //         let updated_state = match client.request(&request) {
//     //             Ok(future) => {
//     //                 match future.await {
//     //                     Ok(response) => {
//     //                         if response.success {

//     // if
// }

// // let mut goal_handle_status = None;

// // let goal = RobotCommand::Goal {
// //     command: "asdf".to_string(),
// // };

// // // let competition_state = competition_state.lock().unwrap().clone();
// // // match competition_state {
// //     // CompetitionState::Started => {
// //         let (goal_handle, result, feedback) = match robot_action_client.send_goal_request(goal)
// //         {
// //             Ok(x) => match x.await {
// //                 Ok(y) => y,
// //                 Err(e) => {
// //                     r2r::log_info!(NODE_ID, "Could not send goal request.");
// //                     return Err(Box::new(e));
// //                 }
// //             },
// //             Err(e) => {
// //                 r2r::log_info!(NODE_ID, "Did not get goal.");
// //                 return Err(Box::new(e));
// //             }
// //         };

// //         goal_handle_status = match goal_handle.get_status() {
// //             Ok(status) => match status {
// //                 r2r::GoalStatus::Accepted => {
// //                     // let shared_state_local =
// //                     //     shared_state_local.update("ur_action_state", "executing".to_spvalue());
// //                     // *shared_state.lock().unwrap() = shared_state_local;
// //                     Some("accepted")
// //                 }
// //                 _ => {
// //                     // let shared_state_local =
// //                     //     shared_state_local.update("ur_action_state", "failed".to_spvalue());
// //                     // *shared_state.lock().unwrap() = shared_state_local;
// //                     None
// //                 }
// //             },
// //             Err(_) => None,
// //         };

// //         match result.await {
// //             Ok((status, msg)) => match status {
// //                 r2r::GoalStatus::Aborted => {
// //                     r2r::log_info!(NODE_ID, "Goal succesfully aborted with: {:?}", msg);
// //                     // let _ = g.publish_feedback(URCommand::Feedback {
// //                     //     current_state: "Goal succesfully aborted.".into(),
// //                     // });
// //                     // Ok(())
// //                 }
// //                 _ => {
// //                     r2r::log_info!(
// //                         NODE_ID,
// //                         "Executing the robot action communication succeeded."
// //                     );
// //                     // let shared_state_local =
// //                     //     shared_state_local.update("ur_action_state", "succeeded".to_spvalue());
// //                     // *shared_state.lock().unwrap() = shared_state_local;
// //                     // let _ = g.publish_feedback(URCommand::Feedback {
// //                     //     current_state: "Executing the Simple Robot Simulator action succeeded.".into(),
// //                     // });
// //                     // Ok(())
// //                 }
// //             },
// //             Err(e) => {}
// //         }
// //     // }
// //     // _ => (),
// // // }
// // Ok(())
// // }

// // pub async fn hl_floor_robot_action_client_ticker(
// //     robot_action_client: &r2r::ActionClient<RobotCommand::Action>,
// //     wait_for_server: impl Future<Output = Result<(), Error>>,
// //     // competition_state: &Arc<Mutex<State>>,
// //     mut timer: r2r::Timer
// // ) -> Result<(), Box<dyn std::error::Error>> {

// // }
