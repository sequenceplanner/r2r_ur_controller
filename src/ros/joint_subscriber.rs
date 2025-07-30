use r2r::sensor_msgs::msg::JointState;
use std::sync::Arc;

use futures::{Stream, StreamExt};
use micro_sp::ToSPValue;
use micro_sp::*;

pub async fn joint_subscriber(
    robot_name: &str,
    mut subscriber: impl Stream<Item = JointState> + Unpin,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut con = connection_manager.get_connection().await;
    loop {
        match subscriber.next().await {
            Some(message) => {
                if let Err(_) = connection_manager.check_redis_health("joint_subscriber").await {
                    continue;
                }
                let joint_states = message.position;

                StateManager::set_sp_value(
                    &mut con,
                    &format!("{robot_name}_joint_states"),
                    &joint_states.to_spvalue(),
                )
                .await;
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
