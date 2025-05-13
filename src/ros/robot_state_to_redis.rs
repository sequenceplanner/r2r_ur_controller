use ordered_float::OrderedFloat;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;
use tokio::sync::mpsc;

use futures::StreamExt;
// use micro_sp::ToSPValue;
use micro_sp::*;
use r2r::QosProfile;

fn tf_to_sp_tf(tf: TransformStamped) -> SPTransformStamped {
    SPTransformStamped {
        active_transform: true,
        enable_transform: true,
        time_stamp: SystemTime::now(),
        parent_frame_id: tf.header.frame_id,
        child_frame_id: tf.child_frame_id,
        transform: SPTransform {
            translation: SPTranslation {
                x: OrderedFloat(tf.transform.translation.x),
                y: OrderedFloat(tf.transform.translation.y),
                z: OrderedFloat(tf.transform.translation.z),
            },
            rotation: SPRotation {
                x: OrderedFloat(tf.transform.rotation.x),
                y: OrderedFloat(tf.transform.rotation.y),
                z: OrderedFloat(tf.transform.rotation.z),
                w: OrderedFloat(tf.transform.rotation.w),
            },
        },
        metadata: MapOrUnknown::UNKNOWN,
    }
}

pub async fn robot_state_to_redis(
    robot_name: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
) -> Result<(), Box<dyn std::error::Error>> {
    let mut subscriber = arc_node
        .lock()
        .unwrap()
        // &format!("{robot_name}_dashboard_server")
        .subscribe::<TFMessage>("/tf", QosProfile::default())?;

    loop {
        match subscriber.next().await {
            Some(message) => {
                for tf in &message.transforms {
                    if tf.child_frame_id == "base_link_inertia" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "base_link_inertia".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "shoulder_link" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "shoulder_link".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "upper_arm_link" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "upper_arm_link".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "forearm_link" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "forearm_link".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "wrist_1_link" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "wrist_1_link".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "wrist_2_link" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "wrist_2_link".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "wrist_3_link" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "wrist_3_link".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "flange" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "flange".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "ft_frame" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "ft_frame".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                    if tf.child_frame_id == "tool0" {
                        state_mgmt
                            .send(StateManagement::MoveTransform(
                                "tool0".to_string(),
                                tf_to_sp_tf(tf.clone()).transform,
                            ))
                            .await?;
                    }
                }
            }
            None => {
                r2r::log_error!(
                    &format!("{robot_name}_joint_subscriber"),
                    "Joint state subscriber did not get the message?"
                );
            }
        }
    }
}
