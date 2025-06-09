use ordered_float::OrderedFloat;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;
use tokio::sync::mpsc;

use futures::{Stream, StreamExt};
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
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
) -> Result<(), Box<dyn std::error::Error>> {
    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;


    for initial in vec![
        ("base_link", "base_link_inertia"),
        ("base_link_inertia", "shoulder_link"),
        ("shoulder_link", "upper_arm_link"),
        ("upper_arm_link", "forearm_link"),
        ("forearm_link", "wrist_1_link"),
        ("wrist_1_link", "wrist_2_link"),
        ("wrist_2_link", "wrist_3_link"),
        ("wrist_3_link", "flange"),
        ("wrist_3_link", "ft_frame"),
        ("flange", "tool0")
        ] {
        state_mgmt
            .send(StateManagement::InsertTransform((
                initial.1.to_string(),
                SPTransformStamped {
                    active_transform: true,
                    enable_transform: true,
                    time_stamp: SystemTime::now(),
                    parent_frame_id: initial.0.to_string(),
                    child_frame_id: initial.1.to_string(),
                    transform: SPTransform::default(),
                    metadata: MapOrUnknown::UNKNOWN,
                }
            )))
            .await?;
    }

    loop {
        
        match subscriber.next().await {
            Some(message) => {
                // log::error!(target: "asdfasdf", "got tf: {:?}", message);
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
