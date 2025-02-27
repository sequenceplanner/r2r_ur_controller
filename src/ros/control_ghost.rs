use micro_sp::ToSPValue;
use micro_sp::*;
use r2r::ur_script_msgs::srv::DashboardCommand as DBCommand;
use r2r::QosProfile;
use r2r_transforms::TransformStamped;
use std::{collections::HashMap, sync::{Arc, Mutex}};
use tokio::sync::{mpsc, oneshot};

use crate::*;

pub const UR_CONTROL_GHOST_TICKER_RATE: u64 = 50;

pub async fn control_ghost(
    robot_name: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>,
    transform_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut timer =
        arc_node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(
                UR_CONTROL_GHOST_TICKER_RATE,
            ))?;

    r2r::log_info!(
        &format!("{robot_name}_control_ghost"),
        "Starting control ghost."
    );

    loop {
        let (response_tx, response_rx) = oneshot::channel();
        state_mgmt
            .send(StateManagement::GetState(response_tx))
            .await?;
        let state = response_rx.await?;

        timer.tick().await?;
    }
}