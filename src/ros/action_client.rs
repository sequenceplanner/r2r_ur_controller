use std::{
    sync::{Arc, Mutex},
    time::SystemTime,
};

// use std::net::TcpStream;
// use std::io;

use futures::StreamExt;
use micro_sp::*;
use r2r::ur_script_msgs::action::ExecuteScript;
use serde::{Deserialize, Serialize};

// use crate::core::structs::{transform_to_string, CommandType, Payload};
use crate::*;

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 250;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0];
pub static DEFAULT_BASEFRAME_ID: &'static str = "base_link"; // base_link if simulation, base if real or ursim
pub static DEFAULT_FACEPLATE_ID: &'static str = "tool0";
// pub static DEFAULT_TCP_ID: &'static str = "svt_tcp";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn action_client(
    _ur_address: &str,
    robot_name: &str,
    _gripper_id: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    connection_manager: &Arc<ConnectionManager>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = &format!("{robot_name}_action_client");
    let client = arc_node
        .lock()
        .unwrap()
        .create_action_client::<ExecuteScript::Action>(&format!("ur_script"))?;
    // let waiting_for_server = r2r::Node::is_available(&client)?;

    let mut timer =
        arc_node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(
                UR_ACTION_SERVER_TICKER_RATE,
            ))?;

    // r2r::log_warn!(
    //     &log_target,
    //     "Waiting for the {robot_name} control action server..."
    // );

    // waiting_for_server.await?; // Maybe we don't need this
    // r2r::log_info!(
    //     &log_target,
    //     "Robot {robot_name} control action server available."
    // );

    let keys: Vec<String> = vec![
        format!("{}_request_trigger", robot_name),
        format!("{}_request_state", robot_name),
        format!("{}_command_type", robot_name),
        format!("{}_accelleration", robot_name),
        format!("{}_velocity", robot_name),
        format!("{}_global_acceleration_scaling", robot_name),
        format!("{}_global_velocity_scaling", robot_name),
        format!("{}_use_execution_time", robot_name),
        format!("{}_execution_time", robot_name),
        format!("{}_use_blend_radius", robot_name),
        format!("{}_blend_radius", robot_name),
        format!("{}_use_joint_positions", robot_name),
        format!("{}_joint_positions", robot_name),
        format!("{}_use_preferred_joint_config", robot_name),
        format!("{}_preferred_joint_config", robot_name),
        format!("{}_use_payload", robot_name),
        format!("{}_payload", robot_name),
        format!("{}_baseframe_id", robot_name),
        format!("{}_faceplate_id", robot_name),
        format!("{}_goal_feature_id", robot_name),
        format!("{}_tcp_id", robot_name),
        format!("{}_root_frame_id", robot_name),
        format!("{}_force_threshold", robot_name),
        format!("{}_use_relative_pose", robot_name),
        format!("{}_relative_pose", robot_name),
        format!("{}_force_feedback", robot_name),
        // format!("{}_command_type", gripper_id),
        // format!("{}_velocity", gripper_id),
        // format!("{}_force", gripper_id),
        // format!("{}_ref_pos_percentage", gripper_id),
    ]
    .iter()
    .map(|k| k.to_string())
    .collect();

    let mut con = connection_manager.get_connection().await;
    'scan: loop {
        timer.tick().await?;
        if let Err(_) = connection_manager.check_redis_health(&log_target).await {
            continue;
        }
        let state = match StateManager::get_state_for_keys(&mut con, &keys).await {
            Some(s) => s,
            None => continue,
        };

        let mut request_trigger = state
            .get_bool_or_default_to_false(&format!("{robot_name}_request_trigger"), &log_target);

        let mut request_state = state
            .get_string_or_default_to_unknown(&format!("{robot_name}_request_state"), &log_target);

        let mut force_feedback = state
            .get_float_or_default_to_zero(&format!("{robot_name}_force_feedback"), &log_target);

        if request_trigger {
            request_trigger = false;
            if request_state == ActionRequestState::Initial.to_string() {
                // let gripper_command_type = state.get_string_or_default_to_unknown(
                //     &format!("{gripper_id}_command_type"),
                //     &log_target,
                // );

                // let gripper_velocity =
                //     state.get_float_or_value(&format!("{gripper_id}_velocity"), 1.0, &log_target);

                // let gripper_force =
                //     state.get_float_or_value(&format!("{gripper_id}_force"), 1.0, &log_target);

                // let gripper_ref_pos_percentage = state.get_int_or_default_to_zero(
                //     &format!("{gripper_id}_ref_pos_percentage"),
                //     &log_target,
                // );

                let command_type = state.get_string_or_default_to_unknown(
                    &format!("{robot_name}_command_type"),
                    &log_target,
                );

                let accelleration = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_accelleration"),
                    &log_target,
                );

                let velocity = state
                    .get_float_or_default_to_zero(&format!("{robot_name}_velocity"), &log_target);

                let global_acceleration_scaling = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_global_acceleration_scaling"),
                    &log_target,
                );

                let global_velocity_scaling = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_global_velocity_scaling"),
                    &log_target,
                );

                let use_execution_time = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_use_execution_time"),
                    &log_target,
                );

                let execution_time = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_execution_time"),
                    &log_target,
                );

                let use_blend_radius = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_use_blend_radius"),
                    &log_target,
                );

                let blend_radius = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_blend_radius"),
                    &log_target,
                );

                let use_joint_positions = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_use_joint_positions"),
                    &log_target,
                );

                let force_threshold = state.get_float_or_default_to_zero(
                    &format!("{robot_name}_force_threshold"),
                    &log_target,
                );

                // let gripper_position = state.get_int_or_default_to_zero(
                //     &format!("{robot_name}_gripper_reference_position"),
                //     &log_target,
                // );

                let joint_positions = if let Some(value) =
                    state.get_value(&format!("{robot_name}_joint_positions"), &log_target)
                {
                    match value {
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
                    }
                } else {
                    SAFE_HOME_JOINT_STATE.to_vec()
                };

                let use_preferred_joint_config = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_use_preferred_joint_config"),
                    &log_target,
                );

                let preferred_joint_config = if let Some(value) =
                    state.get_value(&format!("{robot_name}_preferred_joint_config"), &log_target)
                {
                    match value {
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
                    }
                } else {
                    SAFE_HOME_JOINT_STATE.to_vec()
                };

                let use_payload = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_use_payload"),
                    &log_target,
                );

                let payload = state.get_string_or_value(
                    &format!("{robot_name}_payload"),
                    Payload::default().to_string(),
                    &log_target,
                );

                let baseframe_id = state.get_string_or_value(
                    &format!("{robot_name}_baseframe_id"),
                    DEFAULT_BASEFRAME_ID.to_string(),
                    &log_target,
                );

                let faceplate_id = state.get_string_or_value(
                    &format!("{robot_name}_faceplate_id"),
                    DEFAULT_FACEPLATE_ID.to_string(),
                    &log_target,
                );

                let goal_feature_id = state.get_string_or_default_to_unknown(
                    &format!("{robot_name}_goal_feature_id"),
                    &log_target,
                );

                let tcp_id = state
                    .get_string_or_default_to_unknown(&format!("{robot_name}_tcp_id"), &log_target);

                let _root_frame_id = state.get_string_or_value(
                    &format!("{robot_name}_root_frame_id"),
                    DEFAULT_ROOT_FRAME_ID.to_string(),
                    &log_target,
                );

                let use_relative_pose = state.get_bool_or_default_to_false(
                    &format!("{robot_name}_use_relative_pose"),
                    &log_target,
                );

                let relative_pose = state.get_string_or_value(
                    &format!("{robot_name}_relative_pose"),
                    "p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]".to_string(),
                    log_target,
                );

                // if command_type != "gripper_move"
                //     && command_type != "gripper_activate"
                //     && command_type != "gripper_open"
                //     && command_type != "gripper_close"
                // {

                let mut target_in_base = transform_to_string(&SPTransformStamped {
                    active_transform: true,
                    enable_transform: true,
                    time_stamp: SystemTime::now(),
                    parent_frame_id: "".to_string(),
                    child_frame_id: "".to_string(),
                    transform: SPTransform::default(),
                    metadata: MapOrUnknown::UNKNOWN,
                });
                let mut tcp_in_faceplate = target_in_base.clone();
                if !use_joint_positions && !use_relative_pose {
                    target_in_base = match TransformsManager::lookup_transform(
                        &mut con,
                        &baseframe_id,
                        &goal_feature_id,
                    )
                    .await
                    {
                        Ok(transform) => transform_to_string(&transform),
                        Err(_) => continue 'scan,
                    };

                    tcp_in_faceplate =
                        match TransformsManager::lookup_transform(&mut con, &faceplate_id, &tcp_id)
                            .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => continue 'scan,
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
                    force_threshold,
                    relative_pose,
                };

                let script = match generate_script(robot_name, robot_command, templates) {
                    Ok(script) => script,
                    Err(_) => {
                        r2r::log_error!("robot", "Failed to generate UR Script.");

                        continue 'scan;
                    }
                };

                let goal = ExecuteScript::Goal { script };

                let (_goal_handle, result, mut feedback) = match client.send_goal_request(goal) {
                    Ok(x) => match x.await {
                        Ok(y) => y,
                        Err(e) => {
                            r2r::log_info!(
                                &format!("{}_ur_controller", robot_name),
                                "Could not send goal request."
                            );
                            return Err(Box::new(e));
                        }
                    },
                    Err(e) => {
                        r2r::log_info!(
                            &format!("{}_ur_controller", robot_name),
                            "Did not get goal."
                        );
                        return Err(Box::new(e));
                    }
                };

                // let connection_manager_clone = connection_manager.clone();
                // tokio::spawn(async move {
                //     while let Some(msg) = feedback.next().await {
                //         println!("got feedback msg: {}", msg.feedback);
                //         let json_string = &msg.feedback;

                //         // Use `if let` for more concise pattern matching when you only handle the `Ok` case.
                //         if let Ok(UrScriptFeedback::Force(force_data)) =
                //             serde_json::from_str::<UrScriptFeedback>(json_string)
                //         {
                //             r2r::log_info!(
                //                 "ur_controller", // The &format! is not needed for a string literal
                //                 "Received Force Feedback: {}",
                //                 force_data
                //             );

                //             // No more cloning inside the loop!
                //             // `connection_manager` is already in scope.
                //             let mut con = connection_manager_clone.get_connection().await;

                //             // Assuming `force_feedback` is a variable that needs to be updated.
                //             let force_feedback = force_data;

                //             StateManager::set_sp_value(
                //                 &mut con,
                //                 "force_feedback",
                //                 &force_feedback.to_spvalue(),
                //             )
                //             .await;
                //         }
                //         // The `Err` cases from `serde_json::from_str` and the `UrScriptFeedback` enum are implicitly ignored.
                //     }
                // });

                let connection_manager_clone = connection_manager.clone();
                tokio::spawn(async move {
                    while let Some(msg) = feedback.next().await {
                        println!("got feedback msg: {}", msg.feedback);
                        let feedback_string = &msg.feedback;

                        // Check if the string starts with "FORCE: "
                        if let Some(value_str) = feedback_string.strip_prefix("FORCE: ") {
                            // If it does, parse the rest of the string into a f64
                            if let Ok(force_data) = value_str.trim().parse::<f64>() {
                                r2r::log_info!(
                                    "ur_controller",
                                    "Received Force Feedback: {}",
                                    force_data
                                );

                                // No more cloning inside the loop!
                                // `connection_manager` is already in scope.
                                let mut con = connection_manager_clone.get_connection().await;

                                // The parsed f64 is now assigned to force_feedback
                                let force_feedback = force_data;

                                StateManager::set_sp_value(
                                    &mut con,
                                    "force_feedback",
                                    &force_feedback.to_spvalue(),
                                )
                                .await;
                            }
                        }
                    }
                });

                match result.await {
                    Ok((status, msg)) => match status {
                        r2r::GoalStatus::Aborted => {
                            r2r::log_error!(
                                &format!("{}_ur_controller", robot_name),
                                "Goal aborted, result is {}.",
                                msg.ok
                            );
                            request_state = ActionRequestState::Failed.to_string();
                        }
                        _ => {
                            r2r::log_info!(
                                &format!("{}_ur_controller", robot_name),
                                "Goal succeeded, result is {}.",
                                msg.ok
                            );
                            request_state = ActionRequestState::Succeeded.to_string();
                        }
                    },
                    Err(e) => {
                        r2r::log_error!(
                            &format!("{}_ur_controller", robot_name),
                            "Goal failed with {}.",
                            e
                        );
                        request_state = ActionRequestState::Failed.to_string();
                    }
                }
            }

            StateManager::set_sp_value(
                &mut con,
                &format!("{robot_name}_request_state"),
                &request_state.to_spvalue(),
            )
            .await;
            StateManager::set_sp_value(
                &mut con,
                &format!("{robot_name}_request_trigger"),
                &request_trigger.to_spvalue(),
            )
            .await;
        }
    }
}

// fn send_gripper_script(host: &str, port: u16, script_content: &str) -> io::Result<u64> {
//     let server_address = format!("{}:{}", host, port);
//     let mut stream = TcpStream::connect(server_address)?;
//     let mut reader = script_content.as_bytes();
//     io::copy(&mut reader, &mut stream)
// }

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
                    &format!("{}_ur_controller", robot_name),
                    "Creating a Tera Context from a serialized Interpretation failed with: {e}.",
                );
                // Err(Box::new(e))
                r2r::log_error!(
                    &format!("{}_ur_controller", robot_name),
                    "An empty Tera Context will be used instead."
                );
                &empty_context
            }
        },
    ) {
        Ok(script) => Ok(script),
        Err(e) => {
            r2r::log_error!(
                &format!("{}_ur_controller", robot_name),
                "Rendering the {}.script Tera Template failed with: {}.",
                robot_command.command_type,
                e
            );
            return Err(Box::new(e));
        }
    }
}

// fn generate_gripper_script(
//     gripper_command: GripperCommand,
//     templates: &tera::Tera,
//     log_target: &str,
// ) -> Result<String, Box<dyn std::error::Error>> {
//     let empty_context = tera::Context::new();
//     match templates.render(
//         &format!("{}.script", gripper_command.command_type.to_string()),
//         match &tera::Context::from_serialize(gripper_command.clone()) {
//             Ok(context) => context,
//             Err(e) => {
//                 log::error!(target: &log_target,
//                     "Creating a Tera Context from a serialized interpretation failed with: {e}.");
//                 log::error!(target: &log_target,
//                     "An empty Tera Context will be used instead.");
//                 &empty_context
//             }
//         },
//     ) {
//         Ok(script) => Ok(script),
//         Err(e) => {
//             log::error!(target: &log_target,
//                 "Rendering the {}.script Tera Template failed with: {}.",
//                 gripper_command.command_type,
//                 e
//             );
//             return Err(Box::new(e));
//         }
//     }
// }
