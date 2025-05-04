use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use futures::stream::StreamExt;
use micro_sp::*;
use r2r::ur_script_msgs::action::ExecuteScript;
// use r2r_transforms::{lookup_transform_with_root, TransformStamped};
use tokio::sync::{mpsc, oneshot};

// use crate::core::structs::{transform_to_string, CommandType, Payload};
use crate::*;

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 200;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
pub static DEFAULT_BASEFRAME_ID: &'static str = "base"; // base_link if simulation, base if real or ursim
pub static DEFAULT_FACEPLATE_ID: &'static str = "flange";
pub static DEFAULT_TCP_ID: &'static str = "tool0";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn action_client(
    robot_name: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
    // transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    // let client = arc_node
    //     .lock()
    //     .unwrap()
    //     .create_action_client::<ExecuteScript::Action>(&format!("{robot_name}_ur_script"))?;
    let client = arc_node
        .lock()
        .unwrap()
        .create_action_client::<ExecuteScript::Action>(&format!("ur_script"))?;
    let waiting_for_server = r2r::Node::is_available(&client)?;
    let mut timer =
        arc_node
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

    'scan: loop {
        let (response_tx, response_rx) = oneshot::channel();
        state_mgmt
            .send(StateManagement::GetState(response_tx))
            .await?;
        let state = response_rx.await?;

        let request_trigger = state.get_bool_or_default_to_false(
            &format!("{robot_name}_action_client"),
            &format!("{robot_name}_request_trigger"),
        );

        let mut request_state = state.get_string_or_default_to_unknown(
            &format!("{robot_name}_action_client"),
            &format!("{robot_name}_request_state"),
        );

        if request_trigger {
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

                let root_frame_id = state.get_string_or_value(
                    &format!("{robot_name}_action_client"),
                    &format!("{robot_name}_root_frame_id"),
                    DEFAULT_ROOT_FRAME_ID.to_string(),
                );

                let (response_tx, response_rx) = oneshot::channel();
                state_mgmt
                    .send(StateManagement::LookupTransform((
                        baseframe_id,
                        goal_feature_id,
                        response_tx,
                    )))
                    .await?;
                let target_in_base_option = response_rx.await?;
                let target_in_base = match target_in_base_option {
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
                let tcp_in_faceplate = match tcp_in_faceplate_option {
                    Some(transform) => transform_to_string(&transform),
                    None => continue 'scan,
                };

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
                        continue 'scan;
                    }
                };

                let goal = ExecuteScript::Goal { script };

                let (goal_handle, result, mut feedback) = match client.send_goal_request(goal) {
                    Ok(future) => match future.await {
                        Ok(triplet) => triplet,
                        Err(e) => {
                            r2r::log_info!(
                                &format!("{robot_name}_action_client"),
                                "Could not send goal request."
                            );
                            return Err(Box::new(e));
                        }
                    },
                    Err(e) => {
                        r2r::log_info!(&format!("{robot_name}_action_client"), "Did not get goal.");
                        return Err(Box::new(e));
                    }
                };

                let mut goal_accepted = false;
                match goal_handle.get_status() {
                    Ok(status) => match status {
                        r2r::GoalStatus::Accepted => {
                            r2r::log_info!(
                                &format!("{robot_name}_action_client"),
                                "Goal was accepted."
                            );
                            goal_accepted = true;
                        }
                        _ => {
                            r2r::log_error!(
                                &format!("{robot_name}_action_client"),
                                "Goal was not accepted."
                            );
                        }
                    },
                    Err(e) => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Failed to get goal status with error: {e}."
                        );
                    }
                }

                if !goal_accepted {
                    request_state = ActionRequestState::Failed.to_string();
                    state_mgmt
                        .send(StateManagement::Set((
                            format!("{robot_name}_request_state"),
                            request_state.to_spvalue(),
                        )))
                        .await?;
                    continue 'scan;
                }

                // spawn a task cancelling a goal if necesarry
                let robot_name_string = robot_name.to_string();
                let robot_name_string_clone = robot_name_string.clone();
                let arc_node_clone = arc_node.clone();
                tokio::task::spawn(async move {
                    let mut inner_timer = arc_node_clone
                        .lock()
                        .unwrap()
                        .create_wall_timer(std::time::Duration::from_millis(
                            UR_ACTION_SERVER_TICKER_RATE,
                        ))
                        .unwrap();

                    'cancel: loop {
                        let cancel_current_goal = state.get_bool_or_default_to_false(
                            &format!("{robot_name_string_clone}_action_client"),
                            &format!("{robot_name_string_clone}_cancel_current_goal"),
                        );
                        // let cancel_current_goal = match state
                        //     .get_value(&format!("{robot_name_string_clone}_cancel_current_goal"))
                        // {
                        //     micro_sp::SPValue::Bool(value) => value,
                        //     _ => {
                        //         r2r::log_error!(
                        //             &format!("{robot_name_string_clone}_action_client"),
                        //             "Couldn't get {} from the shared state.",
                        //             &format!("{robot_name_string_clone}_cancel_current_goal")
                        //         );
                        //         false
                        //     }
                        // };
                        if cancel_current_goal {
                            match goal_handle.cancel() {
                                Ok(cancel_future) => match cancel_future.await {
                                    Ok(()) => {
                                        r2r::log_info!(
                                            &format!("{robot_name_string_clone}_action_client"),
                                            "Goal succesfully cancelled."
                                        );
                                    }
                                    Err(e) => {
                                        r2r::log_info!(
                                            &format!("{robot_name_string_clone}_action_client"),
                                            "Goal unsuccesfully cancelled with error {e}."
                                        );
                                    }
                                },
                                Err(e) => {
                                    r2r::log_info!(
                                        &format!("{robot_name_string_clone}_action_client"),
                                        "Unable to cancel goal with error: {e}"
                                    );
                                }
                            }
                            break 'cancel;
                        }
                        inner_timer.tick().await.expect("Timer errored.");
                    }
                });

                // spawn a task for getting the feedback
                let robot_name_string_clone = robot_name_string.clone();
                tokio::task::spawn(async move {
                    match feedback.next().await {
                        Some(feedback_msg) => {
                            r2r::log_info!(
                                &format!("{robot_name_string_clone}_action_client"),
                                "Got feedback from UR Script Driver: {:?}.",
                                feedback_msg
                            );
                        }
                        None => (),
                    }
                });

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
