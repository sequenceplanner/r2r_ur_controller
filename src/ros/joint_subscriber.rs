use r2r::sensor_msgs::msg::JointState;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

use futures::{Stream, StreamExt};
use micro_sp::ToSPValue;
use micro_sp::*;
use r2r::QosProfile;

pub async fn joint_subscriber(
    robot_name: &str,
    mut subscriber: impl Stream<Item = JointState> + Unpin,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
) -> Result<(), Box<dyn std::error::Error>> {
    // tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
    // let mut subscriber = arc_node
    //     .lock()
    //     .unwrap()
    //     // &format!("{robot_name}_dashboard_server")
    //     .subscribe::<JointState>("joint_states", QosProfile::default())?;

    loop {
        match subscriber.next().await {
            Some(message) => {
                let joint_states = message.position;

                state_mgmt
                    .send(StateManagement::Set((
                        format!("{robot_name}_joint_states"),
                        joint_states.to_spvalue(),
                    )))
                    .await?;
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
