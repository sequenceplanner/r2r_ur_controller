use std::{sync::{Arc, Mutex}, time::SystemTime};

use futures::stream::StreamExt;
use micro_sp::*;
use r2r::ur_script_msgs::action::ExecuteScript;
// use r2r_transforms::{lookup_transform_with_root, TransformStamped};
use tokio::sync::{mpsc, oneshot};

// use crate::core::structs::{transform_to_string, CommandType, Payload};
use crate::*;

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 200;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0];
pub static DEFAULT_BASEFRAME_ID: &'static str = "base_link"; // base_link if simulation, base if real or ursim
pub static DEFAULT_FACEPLATE_ID: &'static str = "tool0";
// pub static DEFAULT_TCP_ID: &'static str = "svt_tcp";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn action_client(
    robot_name: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
    // transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    // tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
    // let client = arc_node
    //     .lock()
    //     .unwrap()
    //     .create_action_client::<ExecuteScript::Action>(&format!("{robot_name}_ur_script"))?;
    let client = arc_node
        .lock()
        .unwrap()
        .create_action_client::<ExecuteScript::Action>(&format!("ur_script"))?;
    // let waiting_for_server = r2r::Node::is_available(&client)?;

    r2r::log_warn!(
        &format!("{robot_name}_action_client"),
        "Waiting for the {robot_name} control action server..."
    );

    // waiting_for_server.await?;
    r2r::log_info!(
        &format!("{robot_name}_action_client"),
        "Robot {robot_name} control action server available."
    );

    let mut timer =
        arc_node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(
                UR_ACTION_SERVER_TICKER_RATE,
            ))?;

    'scan: loop {
        let (response_tx, response_rx) = oneshot::channel();
        state_mgmt
            .send(StateManagement::GetState(response_tx))
            .await?;
        let state = response_rx.await?;

        let mut request_trigger = state.get_bool_or_default_to_false(
            &format!("{robot_name}_action_client"),
            &format!("{robot_name}_request_trigger"),
        );

        let mut request_state = state.get_string_or_default_to_unknown(
            &format!("{robot_name}_action_client"),
            &format!("{robot_name}_request_state"),
        );

        if request_trigger {
            request_trigger = false;
            if request_state == ActionRequestState::Initial.to_string() {
                let command_type = state.get_string_or_default_to_unknown(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_command_type"),
                );

                let accelleration = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_accelleration"),
                );

                let velocity = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_velocity"),
                );

                let global_acceleration_scaling = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_global_acceleration_scaling"),
                );

                let global_velocity_scaling = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_global_velocity_scaling"),
                );

                let use_execution_time = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_use_execution_time"),
                );

                let execution_time = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_execution_time"),
                );

                let use_blend_radius = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_use_blend_radius"),
                );

                let blend_radius = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_blend_radius"),
                );

                let use_joint_positions = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_use_joint_positions"),
                );

                let joint_positions =
                    match state.get_value(&format!("{robot_name}_joint_positions")) {
                        micro_sp::SPValue::Array(array_or_unknown) => match array_or_unknown {
                            ArrayOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE.to_vec(),
                            ArrayOrUnknown::Array(values) => values
                                .iter()
                                .enumerate()
                                .map(|(i, val)| match val {
                                    micro_sp::SPValue::Float64(float_or_unknown) => {
                                        match float_or_unknown {
                                            FloatOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE[i],
                                            FloatOrUnknown::Float64(ordered_float) => {
                                                ordered_float.into_inner()
                                            }
                                        }
                                    }
                                    _ => SAFE_HOME_JOINT_STATE[i],
                                })
                                .collect(),
                        },
                        _ => SAFE_HOME_JOINT_STATE.to_vec(),
                    };

                let use_preferred_joint_config = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_use_preferred_joint_config"),
                );

                let preferred_joint_config =
                    match state.get_value(&format!("{robot_name}_preferred_joint_config")) {
                        micro_sp::SPValue::Array(array_or_unknown) => match array_or_unknown {
                            ArrayOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE.to_vec(),
                            ArrayOrUnknown::Array(values) => values
                                .iter()
                                .enumerate()
                                .map(|(i, val)| match val {
                                    micro_sp::SPValue::Float64(float_or_unknown) => {
                                        match float_or_unknown {
                                            FloatOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE[i],
                                            FloatOrUnknown::Float64(ordered_float) => {
                                                ordered_float.into_inner()
                                            }
                                        }
                                    }
                                    _ => SAFE_HOME_JOINT_STATE[i],
                                })
                                .collect(),
                        },
                        _ => SAFE_HOME_JOINT_STATE.to_vec(),
                    };

                let use_payload = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_use_payload"),
                );

                let payload = state.get_string_or_value(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_payload"),
                    Payload::default().to_string(),
                );

                let baseframe_id = state.get_string_or_value(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_baseframe_id"),
                    DEFAULT_BASEFRAME_ID.to_string(),
                );

                let faceplate_id = state.get_string_or_value(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_faceplate_id"),
                    DEFAULT_FACEPLATE_ID.to_string(),
                );

                let goal_feature_id = state.get_string_or_default_to_unknown(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_goal_feature_id"),
                );

                let tcp_id = state.get_string_or_default_to_unknown(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_tcp_id"),
                );

                let _root_frame_id = state.get_string_or_value(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_root_frame_id"),
                    DEFAULT_ROOT_FRAME_ID.to_string(),
                );

                let mut target_in_base = transform_to_string(&SPTransformStamped {
                    active_transform: true,
                    enable_transform: true,
                    time_stamp: SystemTime::now(),
                    parent_frame_id: "".to_string(),
                    child_frame_id: "".to_string(),
                    transform: SPTransform::default(),
                    metadata: MapOrUnknown::UNKNOWN
                });
                let mut tcp_in_faceplate = target_in_base.clone();
                if !use_joint_positions {
                    let (response_tx, response_rx) = oneshot::channel();
                    state_mgmt
                        .send(StateManagement::LookupTransform((
                            baseframe_id,
                            goal_feature_id,
                            response_tx,
                        )))
                        .await?;
                    let target_in_base_option = response_rx.await?;
                    target_in_base = match target_in_base_option {
                        Some(transform) => transform_to_string(&transform),
                        None => continue 'scan,
                    };

                    let (response_tx, response_rx) = oneshot::channel();
                    state_mgmt
                        .send(StateManagement::LookupTransform((
                            faceplate_id,
                            tcp_id,
                            response_tx,
                        )))
                        .await?;
                    let tcp_in_faceplate_option = response_rx.await?;
                    tcp_in_faceplate = match tcp_in_faceplate_option {
                        Some(transform) => transform_to_string(&transform),
                        None => continue 'scan,
                    };
                }

                let robot_command = RobotCommand {
                    command_type,
                    accelleration,
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
                    use_payload,
                    payload,
                    target_in_base,
                    tcp_in_faceplate,
                };

                let script = match generate_script(robot_name, robot_command, templates) {
                    Ok(script) => script,
                    Err(_) => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Failed to generate UR Script."
                        );
                        // Inform state that the request has failed and pull down the trigger
                        // request_state = ActionRequestState::Failed.to_string();
                        // state_mgmt
                        //     .send(StateManagement::Set((
                        //         format!("{robot_name}_request_state"),
                        //         request_state.to_spvalue(),
                        //     )))
                        //     .await?;
                        // state_mgmt
                        // .send(StateManagement::Set((
                        //     format!("{robot_name}_request_trigger"),
                        //     false.to_spvalue(),
                        // )))
                        // .await?;

                        continue 'scan;
                    }
                };

                let goal = ExecuteScript::Goal { script };

                // let (goal_handle, result, mut feedback) = match client.send_goal_request(goal) {
                //     Ok(future) => match future.await {
                //         Ok(triplet) => triplet,
                //         Err(e) => {
                //             r2r::log_error!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Could not send goal request with error: {e}."
                //             );
                //             state_mgmt
                //                 .send(StateManagement::Set((
                //                     format!("{robot_name}_request_state"),
                //                     ActionRequestState::Failed.to_string().to_spvalue(),
                //                 )))
                //                 .await?;
                //             state_mgmt
                //                 .send(StateManagement::Set((
                //                     format!("{robot_name}_request_trigger"),
                //                     false.to_spvalue(),
                //                 )))
                //                 .await?;
                //             continue 'scan;
                //             // return Err(Box::new(e));
                //         }
                //     },
                //     Err(e) => {
                //         r2r::log_info!(
                //             &format!("{robot_name}_action_client"),
                //             "Did not get goal with error: {e}."
                //         );
                //         state_mgmt
                //             .send(StateManagement::Set((
                //                 format!("{robot_name}_request_state"),
                //                 ActionRequestState::Failed.to_string().to_spvalue(),
                //             )))
                //             .await?;
                //         state_mgmt
                //             .send(StateManagement::Set((
                //                 format!("{robot_name}_request_trigger"),
                //                 false.to_spvalue(),
                //             )))
                //             .await?;
                //         continue 'scan;
                //     }
                // };

                // let mut goal_accepted = false;
                // match goal_handle.get_status() {
                //     Ok(status) => match status {
                //         r2r::GoalStatus::Accepted => {
                //             r2r::log_info!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal status was accepted."
                //             );
                //             goal_accepted = true;
                //         }
                //         r2r::GoalStatus::Executing => {
                //             r2r::log_info!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal status is executing."
                //             );
                //         }
                //         r2r::GoalStatus::Succeeded => {
                //             r2r::log_info!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal status is succeeded."
                //             );
                //         }
                //         r2r::GoalStatus::Unknown => {
                //             r2r::log_warn!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal status is unknown."
                //             );
                //         }
                //         r2r::GoalStatus::Canceled => {
                //             r2r::log_info!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal is cancelled."
                //             );
                //         }
                //         r2r::GoalStatus::Canceling => {
                //             r2r::log_info!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal is canceling."
                //             );
                //         }
                //         r2r::GoalStatus::Aborted => {
                //             r2r::log_info!(
                //                 &format!("{robot_name}_action_client"),
                //                 "Goal is canceling."
                //             );
                //         }
                //     },
                //     Err(e) => {
                //         r2r::log_error!(
                //             &format!("{robot_name}_action_client"),
                //             "Failed to get goal status with error: {e}."
                //         );
                //     }
                // }

                // if !goal_accepted {
                //     state_mgmt
                //         .send(StateManagement::Set((
                //             format!("{robot_name}_request_state"),
                //             ActionRequestState::Failed.to_string().to_spvalue(),
                //         )))
                //         .await?;
                //     state_mgmt
                //         .send(StateManagement::Set((
                //             format!("{robot_name}_request_trigger"),
                //             false.to_spvalue(),
                //         )))
                //         .await?;
                //     continue 'scan;
                // }

                // // spawn a task cancelling a goal if necesarry
                // let robot_name_string = robot_name.to_string();
                // let robot_name_string_clone = robot_name_string.clone();
                // let arc_node_clone = arc_node.clone();
                // tokio::task::spawn(async move {
                //     let mut inner_timer = arc_node_clone
                //         .lock()
                //         .unwrap()
                //         .create_wall_timer(std::time::Duration::from_millis(
                //             UR_ACTION_SERVER_TICKER_RATE,
                //         ))
                //         .unwrap();

                //     'cancel: loop {
                //         let cancel_current_goal = state.get_bool_or_default_to_false(
                //             &format!("{robot_name_string_clone}_action_client"),
                //             &format!("{robot_name_string_clone}_cancel_current_goal"),
                //         );
                //         // let cancel_current_goal = match state
                //         //     .get_value(&format!("{robot_name_string_clone}_cancel_current_goal"))
                //         // {
                //         //     micro_sp::SPValue::Bool(value) => value,
                //         //     _ => {
                //         //         r2r::log_error!(
                //         //             &format!("{robot_name_string_clone}_action_client"),
                //         //             "Couldn't get {} from the shared state.",
                //         //             &format!("{robot_name_string_clone}_cancel_current_goal")
                //         //         );
                //         //         false
                //         //     }
                //         // };
                //         if cancel_current_goal {
                //             match goal_handle.cancel() {
                //                 Ok(cancel_future) => match cancel_future.await {
                //                     Ok(()) => {
                //                         r2r::log_info!(
                //                             &format!("{robot_name_string_clone}_action_client"),
                //                             "Goal succesfully cancelled."
                //                         );
                //                     }
                //                     Err(e) => {
                //                         r2r::log_info!(
                //                             &format!("{robot_name_string_clone}_action_client"),
                //                             "Goal unsuccesfully cancelled with error {e}."
                //                         );
                //                     }
                //                 },
                //                 Err(e) => {
                //                     r2r::log_info!(
                //                         &format!("{robot_name_string_clone}_action_client"),
                //                         "Unable to cancel goal with error: {e}"
                //                     );
                //                 }
                //             }
                //             break 'cancel;
                //         }
                //         inner_timer.tick().await.expect("Timer errored.");
                //     }
                // });

                // // spawn a task for getting the feedback
                // let robot_name_string_clone = robot_name_string.clone();
                // tokio::task::spawn(async move {
                //     match feedback.next().await {
                //         Some(feedback_msg) => {
                //             r2r::log_info!(
                //                 &format!("{robot_name_string_clone}_action_client"),
                //                 "Got feedback from UR Script Driver: {:?}.",
                //                 feedback_msg
                //             );
                //         }
                //         None => (),
                //     }
                // });

                let (goal_handle, result, mut feedback) = match client.send_goal_request(goal) {
                    Ok(x) => match x.await {
                        Ok(y) => y,
                        Err(e) => {
                            r2r::log_info!("asdf", "Could not send goal request.");
                            return Err(Box::new(e));
                        }
                    },
                    Err(e) => {
                        r2r::log_info!("asdf", "Did not get goal.");
                        return Err(Box::new(e));
                    }
                };          

                // match result.await {
                //     Ok((status, msg)) => match status {
                //         r2r::GoalStatus::Aborted => {
                //             r2r::log_info!("asdf", "Goal succesfully aborted with: {:?}", msg);
                //             current_request_status_str = ActionRequestState::Failed.to_string();
                //             // Ok(ActionResult::Bool(false))
                //         }
                //         _ => {
                //             r2r::log_info!("asdf", "Executing the UR Script succeeded? {}", msg.ok);
                //             current_request_status_str = ActionRequestState::Succeeded.to_string();
                //             // Ok(ActionResult::Bool(msg.ok))
                //         }
                //     },
                //     Err(e) => {
                //         r2r::log_error!("asdf", "UR Script Driver Action failed with: {:?}", e,);
                //         return Err(Box::new(e));
                //     }
                // }

                match result.await {
                    Ok((status, msg)) => match status {
                        r2r::GoalStatus::Aborted => {
                            r2r::log_error!(
                                &format!("{robot_name}_action_client"),
                                "Goal aborted, result is {}.",
                                msg.ok
                            );
                            request_state = ActionRequestState::Failed.to_string();
                        }
                        _ => {
                            r2r::log_info!(
                                &format!("{robot_name}_action_client"),
                                "Goal succeeded, result is {}.",
                                msg.ok
                            );
                            request_state = ActionRequestState::Succeeded.to_string();
                        }
                    },
                    Err(e) => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Goal failed with {}.",
                            e
                        );
                        request_state = ActionRequestState::Failed.to_string();
                    }
                }
            }

            state_mgmt
                .send(StateManagement::Set((
                    format!("{robot_name}_request_state"),
                    request_state.to_spvalue(),
                )))
                .await?;

                state_mgmt
                .send(StateManagement::Set((
                    format!("{robot_name}_request_trigger"),
                    request_trigger.to_spvalue(),
                )))
                .await?;
        }
        timer.tick().await?;
    }
}

fn generate_script(
    robot_name: &str,
    robot_command: RobotCommand,
    templates: &tera::Tera,
) -> Result<String, Box<dyn std::error::Error>> {
    // ) -> Option<String> {
    let empty_context = tera::Context::new();
    match templates.render(
        &format!("{}.script", robot_command.command_type.to_string()),
        match &tera::Context::from_serialize(robot_command.clone()) {
            Ok(context) => context,
            Err(e) => {
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "Creating a Tera Context from a serialized Interpretation failed with: {e}.",
                );
                // Err(Box::new(e))
                r2r::log_error!(
                    &format!("{robot_name}_action_client"),
                    "An empty Tera Context will be used instead."
                );
                &empty_context
            }
        },
    ) {
        Ok(script) => Ok(script),
        Err(e) => {
            r2r::log_error!(
                &format!("{robot_name}_action_client"),
                "Rendering the {}.script Tera Template failed with: {}.",
                robot_command.command_type,
                e
            );
            return Err(Box::new(e));
        }
    }
}




























// use std::{
//     sync::{Arc, Mutex},
//     time::SystemTime,
// };

// use futures::{future::{self, Either}, stream::StreamExt};
// use micro_sp::*;
// use r2r::ur_script_msgs::action::ExecuteScript;
// // use r2r_transforms::{lookup_transform_with_root, TransformStamped};
// use tokio::sync::{mpsc, oneshot};

// // use crate::core::structs::{transform_to_string, CommandType, Payload};
// use crate::*;

// pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 200;
// pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0];
// pub static DEFAULT_BASEFRAME_ID: &'static str = "base_link";
// pub static DEFAULT_FACEPLATE_ID: &'static str = "tool0";
// pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

// pub async fn action_client(
//     robot_name: &str,
//     arc_node: Arc<Mutex<r2r::Node>>,
//     state_mgmt: mpsc::Sender<StateManagement>,
//     templates: &tera::Tera,
// ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//     let client = arc_node
//         .lock()
//         .unwrap()
//         .create_action_client::<ExecuteScript::Action>(&format!("ur_script"))?; // Assuming "ur_script" is correct, was {robot_name}_ur_script

//     let waiting_for_server = r2r::Node::is_available(&client)?;

//     r2r::log_warn!(
//         &format!("{robot_name}_action_client"),
//         "Waiting for the {} control action server ('ur_script')...",
//         robot_name
//     );

//     waiting_for_server.await?;
//     r2r::log_info!(
//         &format!("{robot_name}_action_client"),
//         "Robot {} control action server available.",
//         robot_name
//     );

//     let mut timer =
//         arc_node
//             .lock()
//             .unwrap()
//             .create_wall_timer(std::time::Duration::from_millis(
//                 UR_ACTION_SERVER_TICKER_RATE,
//             ))?;

//     'scan: loop {
//         // Fetch current state
//         let (response_tx, response_rx) = oneshot::channel();
//         if state_mgmt
//             .send(StateManagement::GetState(response_tx))
//             .await
//             .is_err()
//         {
//             r2r::log_error!(
//                 &format!("{robot_name}_action_client"),
//                 "Failed to send GetState request to state_mgmt. Exiting loop."
//             );
//             break 'scan;
//         }
//         let state = match response_rx.await {
//             Ok(s) => s,
//             Err(e) => {
//                 r2r::log_error!(
//                     &format!("{robot_name}_action_client"),
//                     "Failed to receive state from state_mgmt: {}. Exiting loop.",
//                     e
//                 );
//                 break 'scan;
//             }
//         };

//         let request_trigger = state.get_bool_or_default_to_false(
//             &format!("{robot_name}_action_client"),
//             &format!("{robot_name}_request_trigger"),
//         );

//         // Mutable request_state, to be updated based on action outcome
//         let mut current_request_status_str = state.get_string_or_default_to_unknown(
//             &format!("{robot_name}_action_client"),
//             &format!("{robot_name}_request_state"),
//         );

//         if request_trigger {
//             r2r::log_info!(
//                 &format!("{robot_name}_action_client"),
//                 "Request trigger is active. Current request state: {}",
//                 current_request_status_str
//             );

//             if current_request_status_str == ActionRequestState::Initial.to_string() {
//                 // --- Parameter Fetching Start ---
//                 // (Your existing comprehensive parameter fetching logic is assumed to be here)
//                 // For brevity, I'll use placeholders, ensure your full logic is retained.
//                 let command_type = state.get_string_or_default_to_unknown(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_command_type"),
//                 );
//                 let accelleration = state.get_float_or_default_to_zero(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_accelleration"),
//                 );
//                 let velocity = state.get_float_or_default_to_zero(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_velocity"),
//                 );
//                 let global_acceleration_scaling = state.get_float_or_default_to_zero(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_global_acceleration_scaling"),
//                 );
//                 let global_velocity_scaling = state.get_float_or_default_to_zero(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_global_velocity_scaling"),
//                 );
//                 let use_execution_time = state.get_bool_or_default_to_false(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_use_execution_time"),
//                 );
//                 let execution_time = state.get_float_or_default_to_zero(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_execution_time"),
//                 );
//                 let use_blend_radius = state.get_bool_or_default_to_false(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_use_blend_radius"),
//                 );
//                 let blend_radius = state.get_float_or_default_to_zero(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_blend_radius"),
//                 );
//                 let use_joint_positions = state.get_bool_or_default_to_false(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_use_joint_positions"),
//                 );
//                 let joint_positions =
//                     match state.get_value(&format!("{robot_name}_joint_positions")) {
//                         micro_sp::SPValue::Array(array_or_unknown) => match array_or_unknown {
//                             ArrayOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE.to_vec(),
//                             ArrayOrUnknown::Array(values) => values
//                                 .iter()
//                                 .enumerate()
//                                 .map(|(i, val)| match val {
//                                     micro_sp::SPValue::Float64(float_or_unknown) => {
//                                         match float_or_unknown {
//                                             FloatOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE[i],
//                                             FloatOrUnknown::Float64(ordered_float) => {
//                                                 ordered_float.into_inner()
//                                             }
//                                         }
//                                     }
//                                     _ => SAFE_HOME_JOINT_STATE[i],
//                                 })
//                                 .collect(),
//                         },
//                         _ => SAFE_HOME_JOINT_STATE.to_vec(),
//                     };
//                 let use_preferred_joint_config = state.get_bool_or_default_to_false(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_use_preferred_joint_config"),
//                 );
//                 let preferred_joint_config =
//                     match state.get_value(&format!("{robot_name}_preferred_joint_config")) {
//                         micro_sp::SPValue::Array(array_or_unknown) => match array_or_unknown {
//                             ArrayOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE.to_vec(),
//                             ArrayOrUnknown::Array(values) => values
//                                 .iter()
//                                 .enumerate()
//                                 .map(|(i, val)| match val {
//                                     micro_sp::SPValue::Float64(float_or_unknown) => {
//                                         match float_or_unknown {
//                                             FloatOrUnknown::UNKNOWN => SAFE_HOME_JOINT_STATE[i],
//                                             FloatOrUnknown::Float64(ordered_float) => {
//                                                 ordered_float.into_inner()
//                                             }
//                                         }
//                                     }
//                                     _ => SAFE_HOME_JOINT_STATE[i],
//                                 })
//                                 .collect(),
//                         },
//                         _ => SAFE_HOME_JOINT_STATE.to_vec(),
//                     };
//                 let use_payload = state.get_bool_or_default_to_false(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_use_payload"),
//                 );
//                 let payload = state.get_string_or_value(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_payload"),
//                     Payload::default().to_string(),
//                 );
//                 let baseframe_id = state.get_string_or_value(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_baseframe_id"),
//                     DEFAULT_BASEFRAME_ID.to_string(),
//                 );
//                 let faceplate_id = state.get_string_or_value(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_faceplate_id"),
//                     DEFAULT_FACEPLATE_ID.to_string(),
//                 );
//                 let goal_feature_id = state.get_string_or_default_to_unknown(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_goal_feature_id"),
//                 );
//                 let tcp_id = state.get_string_or_default_to_unknown(
//                     &format!("{robot_name}_action_client"),
//                     &format!("{robot_name}_tcp_id"),
//                 );
//                 // --- Parameter Fetching End ---

//                 // --- Transform Logic Start ---
//                 let mut target_in_base = transform_to_string(&SPTransformStamped {
//                     active_transform: true,
//                     enable_transform: true,
//                     time_stamp: SystemTime::now(),
//                     parent_frame_id: "".to_string(),
//                     child_frame_id: "".to_string(),
//                     transform: SPTransform::default(),
//                     metadata: MapOrUnknown::UNKNOWN,
//                 });
//                 let mut tcp_in_faceplate_str = target_in_base.clone();

//                 if !use_joint_positions {
//                     // Lookup target_in_base
//                     let (tib_tx, tib_rx) = oneshot::channel();
//                     state_mgmt
//                         .send(StateManagement::LookupTransform((
//                             baseframe_id.clone(),
//                             goal_feature_id.clone(),
//                             tib_tx,
//                         )))
//                         .await?;
//                     match tib_rx.await? {
//                         Some(transform) => target_in_base = transform_to_string(&transform),
//                         None => {
//                             r2r::log_error!(&format!("{robot_name}_action_client"), "Failed to lookup transform from '{}' to '{}'. Aborting this request.", baseframe_id, goal_feature_id);
//                             current_request_status_str = ActionRequestState::Failed.to_string();
//                             // This state update and trigger reset will happen at the end of the `if request_trigger` block
//                             // However, to prevent further processing in *this* cycle:
//                             state_mgmt
//                                 .send(StateManagement::Set((
//                                     format!("{robot_name}_request_state"),
//                                     current_request_status_str.to_spvalue(),
//                                 )))
//                                 .await?;
//                             state_mgmt
//                                 .send(StateManagement::Set((
//                                     format!("{robot_name}_request_trigger"),
//                                     false.to_spvalue(),
//                                 )))
//                                 .await?;
//                             continue 'scan;
//                         }
//                     }
//                     // Lookup tcp_in_faceplate
//                     let (tif_tx, tif_rx) = oneshot::channel();
//                     state_mgmt
//                         .send(StateManagement::LookupTransform((
//                             faceplate_id.clone(),
//                             tcp_id.clone(),
//                             tif_tx,
//                         )))
//                         .await?;
//                     match tif_rx.await? {
//                         Some(transform) => tcp_in_faceplate_str = transform_to_string(&transform),
//                         None => {
//                             r2r::log_error!(&format!("{robot_name}_action_client"), "Failed to lookup transform from '{}' to '{}'. Aborting this request.", faceplate_id, tcp_id);
//                             current_request_status_str = ActionRequestState::Failed.to_string();
//                             state_mgmt
//                                 .send(StateManagement::Set((
//                                     format!("{robot_name}_request_state"),
//                                     current_request_status_str.to_spvalue(),
//                                 )))
//                                 .await?;
//                             state_mgmt
//                                 .send(StateManagement::Set((
//                                     format!("{robot_name}_request_trigger"),
//                                     false.to_spvalue(),
//                                 )))
//                                 .await?;
//                             continue 'scan;
//                         }
//                     }
//                 }
//                 // --- Transform Logic End ---

//                 let robot_command = RobotCommand {
//                     command_type,
//                     accelleration,
//                     velocity,
//                     global_acceleration_scaling,
//                     global_velocity_scaling,
//                     use_execution_time,
//                     execution_time,
//                     use_blend_radius,
//                     blend_radius,
//                     use_joint_positions,
//                     joint_positions,
//                     use_preferred_joint_config,
//                     preferred_joint_config,
//                     use_payload,
//                     payload,
//                     target_in_base: target_in_base,
//                     tcp_in_faceplate: tcp_in_faceplate_str,
//                 };

//                 let script = match generate_script(robot_name, robot_command, templates) {
//                     Ok(s) => s,
//                     Err(e) => {
//                         r2r::log_error!(
//                             &format!("{robot_name}_action_client"),
//                             "Failed to generate UR Script: {}. Aborting this request.",
//                             e
//                         );
//                         current_request_status_str = ActionRequestState::Failed.to_string();
//                         // State & trigger reset will happen at the end of `if request_trigger` block.
//                         // To ensure it's immediate for this path:
//                         state_mgmt
//                             .send(StateManagement::Set((
//                                 format!("{robot_name}_request_state"),
//                                 current_request_status_str.to_spvalue(),
//                             )))
//                             .await?;
//                         state_mgmt
//                             .send(StateManagement::Set((
//                                 format!("{robot_name}_request_trigger"),
//                                 false.to_spvalue(),
//                             )))
//                             .await?;
//                         continue 'scan;
//                     }
//                 };
//                 r2r::log_info!(
//                     &format!("{robot_name}_action_client"),
//                     "Generated script: {}",
//                     script
//                 );

//                 let goal = ExecuteScript::Goal { script };

//                 r2r::log_info!(
//                     &format!("{robot_name}_action_client"),
//                     "Sending goal to action server..."
//                 );

//                 let (goal_handle, result, mut feedback) = match client.send_goal_request(goal) {
//                     Ok(x) => match x.await {
//                         Ok(y) => y,
//                         Err(e) => {
//                             r2r::log_info!("asdf", "Could not send goal request.");
//                             return Err(Box::new(e));
//                         }
//                     },
//                     Err(e) => {
//                         r2r::log_info!("asdf", "Did not get goal.");
//                         return Err(Box::new(e));
//                     }
//                 };          

//                 match result.await {
//                     Ok((status, msg)) => match status {
//                         r2r::GoalStatus::Aborted => {
//                             r2r::log_info!("asdf", "Goal succesfully aborted with: {:?}", msg);
//                             current_request_status_str = ActionRequestState::Failed.to_string();
//                             // Ok(ActionResult::Bool(false))
//                         }
//                         _ => {
//                             r2r::log_info!("asdf", "Executing the UR Script succeeded? {}", msg.ok);
//                             current_request_status_str = ActionRequestState::Succeeded.to_string();
//                             // Ok(ActionResult::Bool(msg.ok))
//                         }
//                     },
//                     Err(e) => {
//                         r2r::log_error!("asdf", "UR Script Driver Action failed with: {:?}", e,);
//                         return Err(Box::new(e));
//                     }
//                 }

                
//                 // match client.send_goal_request(goal).unwrap().await {
//                 //     Ok((goal_handle, result_future, mut feedback_stream)) => {
//                 //         r2r::log_info!(
//                 //             &format!("{robot_name}_action_client"),
//                 //             "Goal request acknowledged by server. Goal ID: {}",
//                 //             // goal_handle.get_goal_id()
//                 //             "some goal"
//                 //         );
//                 //         current_request_status_str = ActionRequestState::Executing.to_string(); // Update state to Executing
//                 //         state_mgmt
//                 //             .send(StateManagement::Set((
//                 //                 format!("{robot_name}_request_state"),
//                 //                 current_request_status_str.clone().to_spvalue(),
//                 //             )))
//                 //             .await?;

//                 //         // Spawn improved feedback task
//                 //         let rn_feedback = robot_name.to_string();
//                 //         tokio::task::spawn(async move {
//                 //             while let Some(feedback_msg) = feedback_stream.next().await {
//                 //                 r2r::log_info!(
//                 //                     &format!("{}_action_client", rn_feedback),
//                 //                     "Feedback: {:?}",
//                 //                     feedback_msg.feedback
//                 //                 );
//                 //                 // Optionally send feedback to state_mgmt here if needed
//                 //             }
//                 //             r2r::log_info!(
//                 //                 &format!("{}_action_client", rn_feedback),
//                 //                 "Feedback stream ended."
//                 //             );
//                 //         });


                        

//                 //         // Spawn improved cancellation task
//                 //         let rn_cancel = robot_name.to_string();
//                 //         let sm_cancel = state_mgmt.clone();
//                 //         let gh_cancel = goal_handle.clone(); // Clone GoalHandle for the task
//                 //         let arc_node_for_cancel_timer = arc_node.clone();
//                 //         tokio::task::spawn(async move {
//                 //             let mut cancel_timer = arc_node_for_cancel_timer
//                 //                 .lock()
//                 //                 .unwrap()
//                 //                 .create_wall_timer(std::time::Duration::from_millis(
//                 //                     UR_ACTION_SERVER_TICKER_RATE,
//                 //                 ))
//                 //                 .unwrap();
//                 //             'cancel_loop: loop {
//                 //                 match gh_cancel.get_status() {
//                 //                     Ok(status) => {
//                 //                         match status {
//                 //                             r2r::GoalStatus::Succeeded |
//                 //                             r2r::GoalStatus::Aborted |
//                 //                             r2r::GoalStatus::Canceled => {
//                 //                                 r2r::log_info!(&format!("{}_action_client_cancel_task", rn_cancel), "Goal is terminal with status: {:?}. Exiting cancel polling task.", status);
//                 //                                 break 'cancel_loop;
//                 //                             }
//                 //                             _ => {
//                 //                                 // Goal is still active (e.g., Accepted, Executing, Canceling)
//                 //                             }
//                 //                         }
//                 //                     }
//                 //                     Err(e) => {
//                 //                         r2r::log_warn!(&format!("{}_action_client_cancel_task", rn_cancel), "Could not get goal status: {}. Assuming not terminal for this check.", e);
//                 //                     }
//                 //                 }

//                 //                 let (resp_tx, resp_rx) = oneshot::channel();
//                 //                 if sm_cancel
//                 //                     .send(StateManagement::GetState(resp_tx))
//                 //                     .await
//                 //                     .is_err()
//                 //                 {
//                 //                     r2r::log_error!(
//                 //                         &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                         "Failed to send GetState to state_mgmt for cancel check."
//                 //                     );
//                 //                     break 'cancel_loop;
//                 //                 }
//                 //                 let loop_state = match resp_rx.await {
//                 //                     Ok(s) => s,
//                 //                     Err(_) => {
//                 //                         r2r::log_error!(
//                 //                             &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                             "Failed to receive state for cancel check."
//                 //                         );
//                 //                         break 'cancel_loop;
//                 //                     }
//                 //                 };

//                 //                 let should_cancel = loop_state.get_bool_or_default_to_false(
//                 //                     &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                     &format!("{}_cancel_current_goal", rn_cancel),
//                 //                 );

//                 //                 if should_cancel {
//                 //                     r2r::log_info!(
//                 //                         &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                         "Cancel trigger active. Requesting goal cancellation..."
//                 //                     );
//                 //                     match gh_cancel.cancel().unwrap().await {
//                 //                         // Use the cloned GoalHandle
//                 //                         Ok(_) => r2r::log_info!(
//                 //                             &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                             "Goal cancellation requested successfully."
//                 //                         ),
//                 //                         Err(e) => r2r::log_error!(
//                 //                             &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                             "Failed to request goal cancellation: {}",
//                 //                             e
//                 //                         ),
//                 //                     }
//                 //                     // Reset the cancel trigger in shared state
//                 //                     if sm_cancel
//                 //                         .send(StateManagement::Set((
//                 //                             format!("{}_cancel_current_goal", rn_cancel),
//                 //                             false.to_spvalue(),
//                 //                         )))
//                 //                         .await
//                 //                         .is_err()
//                 //                     {
//                 //                         r2r::log_warn!(
//                 //                             &format!("{}_action_client_cancel_task", rn_cancel),
//                 //                             "Could not reset cancel trigger in state."
//                 //                         );
//                 //                     }
//                 //                     break 'cancel_loop; // Exit after attempting cancellation
//                 //                 }
//                 //                 cancel_timer.tick().await.unwrap();
//                 //             }
//                 //         });

//                 //         r2r::log_info!(
//                 //             &format!("{robot_name}_action_client"),
//                 //             "Waiting for goal result..."
//                 //         );
//                 //         match result_future.await {
//                 //             Ok((status, msg)) => {
//                 //                 match status {
//                 //                     r2r::GoalStatus::Succeeded => {
//                 //                         r2r::log_info!(
//                 //                             &format!("{robot_name}_action_client"),
//                 //                             "Goal Succeeded. Result: ok={}",
//                 //                             msg.ok
//                 //                         );
//                 //                         current_request_status_str =
//                 //                             ActionRequestState::Succeeded.to_string();
//                 //                     }
//                 //                     r2r::GoalStatus::Aborted => {
//                 //                         r2r::log_error!(
//                 //                             &format!("{robot_name}_action_client"),
//                 //                             "Goal Aborted. Result: ok={}",
//                 //                             msg.ok
//                 //                         );
//                 //                         current_request_status_str =
//                 //                             ActionRequestState::Failed.to_string();
//                 //                     }
//                 //                     r2r::GoalStatus::Canceled => {
//                 //                         r2r::log_info!(
//                 //                             &format!("{robot_name}_action_client"),
//                 //                             "Goal Canceled. Result: ok={}",
//                 //                             msg.ok
//                 //                         );
//                 //                         current_request_status_str =
//                 //                             ActionRequestState::Failed.to_string();
//                 //                         // Or a dedicated "Cancelled" state
//                 //                     }
//                 //                     _ => {
//                 //                         r2r::log_warn!(&format!("{robot_name}_action_client"), "Goal finished with unexpected status: {:?}. Result: ok={}", status, msg.ok);
//                 //                         current_request_status_str =
//                 //                             ActionRequestState::Failed.to_string();
//                 //                     }
//                 //                 }
//                 //             }
//                 //             Err(e) => {
//                 //                 r2r::log_error!(
//                 //                     &format!("{robot_name}_action_client"),
//                 //                     "Waiting for goal result failed: {}",
//                 //                     e
//                 //                 );
//                 //                 current_request_status_str = ActionRequestState::Failed.to_string();
//                 //             }
//                 //         }
//                 //     }
//                 //     Err(e) => {
//                 //         // Error from client.send_goal_request().await
//                 //         r2r::log_error!(
//                 //             &format!("{robot_name}_action_client"),
//                 //             "Sending goal request failed: {}",
//                 //             e
//                 //         );
//                 //         current_request_status_str = ActionRequestState::Failed.to_string();
//                 //     }
//                 // }
//             } else {
//                 // request_trigger is true, but state is not Initial.
//                 // This implies an issue or a completed action from a previous cycle where the trigger wasn't reset.
//                 r2r::log_warn!(
//                     &format!("{robot_name}_action_client"),
//                     "Request trigger is true, but current request_state is '{}'. Will reset trigger.",
//                     current_request_status_str
//                 );
//                 // The state (`current_request_status_str`) will be written as is, and trigger will be reset.
//             }

//             // --- Consistent State Update and Trigger Reset ---
//             // Update the final status of the request for this cycle.
//             if state_mgmt
//                 .send(StateManagement::Set((
//                     format!("{robot_name}_request_state"),
//                     current_request_status_str.to_spvalue(),
//                 )))
//                 .await
//                 .is_err()
//             {
//                 r2r::log_error!(
//                     &format!("{robot_name}_action_client"),
//                     "Failed to set final request_state."
//                 );
//             }

//             // CRITICAL: Always reset the trigger if it was active, to prevent re-processing.
//             if state_mgmt
//                 .send(StateManagement::Set((
//                     format!("{robot_name}_request_trigger"),
//                     false.to_spvalue(),
//                 )))
//                 .await
//                 .is_err()
//             {
//                 r2r::log_error!(
//                     &format!("{robot_name}_action_client"),
//                     "Failed to reset request_trigger."
//                 );
//             }
//             r2r::log_info!(
//                 &format!("{robot_name}_action_client"),
//                 "Request trigger processed and reset."
//             );
//             // --- End Consistent State Update ---
//         } // End of `if request_trigger`

//         if timer.tick().await.is_err() {
//             r2r::log_error!(
//                 &format!("{robot_name}_action_client"),
//                 "Timer tick failed. Exiting loop."
//             );
//             break 'scan;
//         }
//     }
//     Ok(())
// }

// // generate_script function remains the same as you provided
// // fn generate_script(
// //     robot_name: &str,
// //     robot_command: RobotCommand,
// //     templates: &tera::Tera,
// // ) -> Result<String, Box<dyn std::error::Error + Send + Sync>> {
// //     let empty_context = tera::Context::new();
// //     let context_to_use = match tera::Context::from_serialize(robot_command.clone()) {
// //         Ok(context) => context,
// //         Err(e) => {
// //             r2r::log_error!(
// //                 &format!("{robot_name}_action_client"),
// //                 "Creating a Tera Context from a serialized RobotCommand failed with: {}. Using empty context.",
// //                 e
// //             );
// //             empty_context
// //         }
// //     };

// //     match templates.render(
// //         &format!("{}.script", robot_command.command_type.to_string()),
// //         &context_to_use, // Pass by reference
// //     ) {
// //         Ok(script) => Ok(script),
// //         Err(e) => {
// //             r2r::log_error!(
// //                 &format!("{robot_name}_action_client"),
// //                 "Rendering the {}.script Tera Template failed with: {}.",
// //                 robot_command.command_type,
// //                 e
// //             );
// //             Err(Box::new(e))
// //         }
// //     }
// // }


// fn generate_script(
//     robot_name: &str,
//     robot_command: RobotCommand,
//     templates: &tera::Tera,
// ) -> Result<String, Box<dyn std::error::Error>> {
//     // ) -> Option<String> {
//     let empty_context = tera::Context::new();
//     match templates.render(
//         &format!("{}.script", robot_command.command_type.to_string()),
//         match &tera::Context::from_serialize(robot_command.clone()) {
//             Ok(context) => context,
//             Err(e) => {
//                 r2r::log_error!(
//                     &format!("{robot_name}_action_client"),
//                     "Creating a Tera Context from a serialized Interpretation failed with: {e}.",
//                 );
//                 // Err(Box::new(e))
//                 r2r::log_error!(
//                     &format!("{robot_name}_action_client"),
//                     "An empty Tera Context will be used instead."
//                 );
//                 &empty_context
//             }
//         },
//     ) {
//         Ok(script) => Ok(script),
//         Err(e) => {
//             r2r::log_error!(
//                 &format!("{robot_name}_action_client"),
//                 "Rendering the {}.script Tera Template failed with: {}.",
//                 robot_command.command_type,
//                 e
//             );
//             return Err(Box::new(e));
//         }
//     }
// }

// // async fn execute_urscript_old(
// //     g: ActionServerGoal<URControl::Action>,
// //     mut cancel: impl Stream<Item = r2r::ActionServerCancelRequest> + Unpin,
// //     urc_client: &r2r::ActionClient<ExecuteScript::Action>,
// //     tf_lookup_client: &r2r::Client<LookupTransform::Service>,
// //     get_extra_client: &r2r::Client<GetExtra::Service>,
// //     templates: &tera::Tera,
// // ) -> Result<ActionResult, Box<dyn std::error::Error>> {
// //     let goal = match generate_script(g.goal.clone(), tf_lookup_client, get_extra_client, templates).await {
// //         Some(script) => ExecuteScript::Goal { script },
// //         None => return Ok(ActionResult::Bool(false)), // RETURN ERROR SOMEHOW: Err(std::error::Error::default())
// //     };

// //     r2r::log_info!(NODE_ID, "Sending request to UR Script Driver.");
// //     let _ = g.publish_feedback(URControl::Feedback {
// //         current_state: "Sending request to UR Script Driver.".into(),
// //     });

// //     let (goal_handle, result, mut feedback) = match urc_client.send_goal_request(goal) {
// //         Ok(x) => match x.await {
// //             Ok(y) => y,
// //             Err(e) => {
// //                 r2r::log_info!(NODE_ID, "Could not send goal request.");
// //                 return Err(Box::new(e));
// //             }
// //         },
// //         Err(e) => {
// //             r2r::log_info!(NODE_ID, "Did not get goal.");
// //             return Err(Box::new(e));
// //         }
// //     };

// //     // spawn task for propagating the feedback
// //     let g_clone = g.clone();
// //     tokio::spawn(async move {
// //         loop {
// //             if let Some(fb) = feedback.next().await {
// //                 let passed_on = URControl::Feedback {
// //                     current_state: fb.feedback,
// //                 };
// //                 if let Err(_) = g_clone.publish_feedback(passed_on) {
// //                     // could not publish, probably done...
// //                     break;
// //                 }
// //             } else {
// //                 // sender dropped, we are done.
// //                 break;
// //             }
// //         }
// //     });

// //     match future::select(result, cancel.next()).await {
// //         Either::Left((res, _cancel_stream)) => {
// //             match res {
// //                 Ok((status, msg)) => match status {
// //                     r2r::GoalStatus::Aborted => {
// //                         r2r::log_info!(NODE_ID, "Goal succesfully aborted with: {:?}", msg);
// //                         Ok(ActionResult::Bool(false))
// //                     }
// //                     _ => {
// //                         r2r::log_info!(NODE_ID, "Executing the UR Script succeeded? {}", msg.ok);
// //                         Ok(ActionResult::Bool(msg.ok))
// //                     }
// //                 },
// //                 Err(e) => {
// //                     r2r::log_error!(NODE_ID, "UR Script Driver Action failed with: {:?}", e,);
// //                     return Err(Box::new(e));
// //                 }
// //             }
// //         },
// //         Either::Right((cancel_request, _nominal)) => {
// //             if let Some(cancel_request) = cancel_request {
// //                 // Always accept cancel requests.
// //                 cancel_request.accept();
// //                 match goal_handle.cancel().expect("could not send cancel request").await {
// //                     Ok(()) => {
// //                         r2r::log_info!(NODE_ID, "Goal succesfully cancelled");
// //                         return Ok(ActionResult::Canceled);
// //                     }
// //                     Err(e) => {
// //                         r2r::log_error!(NODE_ID, "Failed to cancel: {}", e);
// //                         return Err(Box::new(e));
// //                     }
// //                 }
// //             } else {
// //                 return Err("Got cancel but its dropped".into());
// //             }
// //         }
// //     }
// // }