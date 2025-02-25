use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use futures::stream::StreamExt;
use micro_sp::*;
use r2r::ur_script_msgs::action::ExecuteScript;
use r2r_transforms::{lookup_transform_with_root, TransformStamped};
use tokio::sync::{mpsc, oneshot};

use crate::core::structs::{transform_to_string, CommandType, Payload};
use crate::*;

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 200;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
pub static DEFAULT_BASEFRAME_ID: &'static str = "base"; // base_link if simulation, base if real or ursim
pub static DEFAULT_FACEPLATE_ID: &'static str = "tool0";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn action_client(
    robot_name: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
    transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    let client = arc_node
        .lock()
        .unwrap()
        .create_action_client::<ExecuteScript::Action>(&format!("{robot_name}_ur_script"))?;
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

        let request_trigger = match state.get_value(&format!("{robot_name}_request_trigger")) {
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
        let mut request_state = match state.get_value(&format!("{robot_name}_request_state")) {
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

        if request_trigger {
            if request_state == ActionRequestState::Initial.to_string() {
                let command_type = match state.get_value(&format!("{robot_name}_command_type")) {
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

                let accelleration = match state.get_value(&format!("{robot_name}_accelleration")) {
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

                let velocity = match state.get_value(&format!("{robot_name}_velocity")) {
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

                let global_acceleration_scaling =
                    match state.get_value(&format!("{robot_name}_global_acceleration_scaling")) {
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
                    match state.get_value(&format!("{robot_name}_global_velocity_scaling")) {
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
                    match state.get_value(&format!("{robot_name}_use_execution_time")) {
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

                let execution_time = match state.get_value(&format!("{robot_name}_execution_time"))
                {
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
                    match state.get_value(&format!("{robot_name}_use_blend_radius")) {
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

                let blend_radius = match state.get_value(&format!("{robot_name}_blend_radius")) {
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
                    match state.get_value(&format!("{robot_name}_use_joint_positions")) {
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

                let joint_positions = match state
                    .get_value(&format!("{robot_name}_joint_positions"))
                {
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

                let use_preferred_joint_config =
                    match state.get_value(&format!("{robot_name}_use_preferred_joint_config")) {
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

                let preferred_joint_config = match state
                    .get_value(&format!("{robot_name}_preferred_joint_config"))
                {
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

                let use_payload = match state.get_value(&format!("{robot_name}_use_payload")) {
                    micro_sp::SPValue::Bool(value) => value,
                    _ => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Couldn't get {} from the shared state.",
                            &format!("{robot_name}_use_payload")
                        );
                        false
                    }
                };

                let payload = match state.get_value(&format!("{robot_name}_payload")) {
                    micro_sp::SPValue::String(value) => value,
                    _ => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Couldn't get {} from the shared state.",
                            &format!("{robot_name}_payload")
                        );
                        Payload::default().to_string()
                    }
                };

                let baseframe_id = match state.get_value(&format!("{robot_name}_baseframe_id")) {
                    micro_sp::SPValue::String(value) => value,
                    _ => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Couldn't get {} from the shared state.",
                            &format!("{robot_name}_baseframe_id")
                        );
                        DEFAULT_BASEFRAME_ID.to_string()
                    }
                };

                let faceplate_id = match state.get_value(&format!("{robot_name}_faceplate_id")) {
                    micro_sp::SPValue::String(value) => value,
                    _ => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Couldn't get {} from the shared state.",
                            &format!("{robot_name}_faceplate_id")
                        );
                        DEFAULT_BASEFRAME_ID.to_string()
                    }
                };

                let goal_feature_id =
                    match state.get_value(&format!("{robot_name}_goal_feature_id")) {
                        micro_sp::SPValue::String(value) => value,
                        _ => {
                            r2r::log_error!(
                                &format!("{robot_name}_action_client"),
                                "Couldn't get {} from the shared state.",
                                &format!("{robot_name}_goal_feature_id")
                            );
                            "unknown".to_string()
                        }
                    };

                let tcp_id = match state.get_value(&format!("{robot_name}_tcp_id")) {
                    micro_sp::SPValue::String(value) => value,
                    _ => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Couldn't get {} from the shared state.",
                            &format!("{robot_name}_tcp_id")
                        );
                        "unknown".to_string()
                    }
                };

                let root_frame_id = match state.get_value(&format!("{robot_name}_root_frame_id")) {
                    micro_sp::SPValue::String(value) => value,
                    _ => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Couldn't get {} from the shared state.",
                            &format!("{robot_name}_root_frame_id")
                        );
                        DEFAULT_ROOT_FRAME_ID.to_string()
                    }
                };

                let target_in_base = match lookup_transform_with_root(
                    &baseframe_id,
                    &goal_feature_id,
                    &root_frame_id,
                    transform_buffer,
                ) {
                    Some(transform) => transform_to_string(&transform),
                    None => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Transform lookup has failed between frames {baseframe_id} and {goal_feature_id}.");
                        continue 'scan;
                    }
                };

                let tcp_in_faceplate = match lookup_transform_with_root(
                    &faceplate_id,
                    &tcp_id,
                    &root_frame_id,
                    transform_buffer,
                ) {
                    Some(transform) => transform_to_string(&transform),
                    None => {
                        r2r::log_error!(
                            &format!("{robot_name}_action_client"),
                            "Transform lookup has failed between frames {faceplate_id} and {tcp_id}.");
                        continue 'scan;
                    }
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
                        let cancel_current_goal = match state
                            .get_value(&format!("{robot_name_string_clone}_cancel_current_goal"))
                        {
                            micro_sp::SPValue::Bool(value) => value,
                            _ => {
                                r2r::log_error!(
                                    &format!("{robot_name_string_clone}_action_client"),
                                    "Couldn't get {} from the shared state.",
                                    &format!("{robot_name_string_clone}_cancel_current_goal")
                                );
                                false
                            }
                        };
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
