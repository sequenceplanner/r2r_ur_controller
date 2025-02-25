use micro_sp::ToSPValue;
use micro_sp::*;
use r2r::ur_script_msgs::srv::DashboardCommand as DBCommand;
use r2r::QosProfile;
use std::sync::{Arc, Mutex};
use tokio::sync::{mpsc, oneshot};

use crate::*;

pub const UR_DASHBOARD_SERVER_TICKER_RATE: u64 = 50;

pub async fn dashboard_client(
    robot_name: &str,
    arc_node: Arc<Mutex<r2r::Node>>,
    state_mgmt: mpsc::Sender<StateManagement>, // instead of &Arc<Mutex<State>>
) -> Result<(), Box<dyn std::error::Error>> {
    // let client = arc_node
    //     .lock()
    //     .unwrap()
    //     .create_client::<DBCommand::Service>(&format!("{robot_name}_dashboard_server"), QosProfile::default())?;
    let client = arc_node
        .lock()
        .unwrap()
        .create_client::<DBCommand::Service>(&format!("dashboard_server"), QosProfile::default())?;
    let waiting_for_server = r2r::Node::is_available(&client)?;
    let mut timer =
        arc_node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(
                UR_DASHBOARD_SERVER_TICKER_RATE,
            ))?;

    r2r::log_warn!(
        &format!("{robot_name}_dashboard_client"),
        "Waiting for the {robot_name} dashboard server..."
    );

    waiting_for_server.await?;
    r2r::log_info!(
        &format!("{robot_name}_dashboard_client"),
        "Robot {robot_name} dashboard server available."
    );

    loop {
        let (response_tx, response_rx) = oneshot::channel();
        state_mgmt
            .send(StateManagement::GetState(response_tx))
            .await?;
        let state = response_rx.await?;

        let request_trigger =
            match state.get_value(&format!("{robot_name}_dashboard_request_trigger")) {
                micro_sp::SPValue::Bool(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_dashboard_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_dashboard_request_trigger")
                    );
                    false
                }
            };
        let mut request_state =
            match state.get_value(&format!("{robot_name}_dashboard_request_state")) {
                micro_sp::SPValue::String(value) => value,
                _ => {
                    r2r::log_error!(
                        &format!("{robot_name}_dashboard_client"),
                        "Couldn't get {} from the shared state.",
                        &format!("{robot_name}_dashboard_request_state")
                    );
                    "unknown".to_string()
                }
            };

        if request_trigger {
            if request_state == ServiceRequestState::Initial.to_string() {
                let command_type =
                    match state.get_value(&format!("{robot_name}_dashboard_command_type")) {
                        micro_sp::SPValue::String(value) => value,
                        _ => {
                            r2r::log_error!(
                                &format!("{robot_name}_dashboard_client"),
                                "Couldn't get {} from the shared state.",
                                &format!("{robot_name}_dashboard_command_type")
                            );
                            DashboardCommandType::UNKNOWN.to_string()
                        }
                    };

                let request = DBCommand::Request { cmd: command_type };
                match client.request(&request) {
                    Ok(future) => match future.await {
                        Ok(response) => {
                            if response.ok {
                                r2r::log_info!(
                                    &format!("{robot_name}_dashboard_client"),
                                    "Dashboard request succeeded."
                                );
                                request_state = ServiceRequestState::Succeeded.to_string();
                            } else {
                                r2r::log_info!(
                                    &format!("{robot_name}_dashboard_client"),
                                    "Dashboard request failed."
                                );
                                request_state = ServiceRequestState::Failed.to_string();
                            }
                        }
                        Err(e) => {
                            r2r::log_info!(
                                &format!("{robot_name}_dashboard_client"),
                                "Dashboard request failed with {e}."
                            );
                            request_state = ServiceRequestState::Failed.to_string();
                        }
                    },
                    Err(e) => {
                        r2r::log_info!(
                            &format!("{robot_name}_dashboard_client"),
                            "Dashboard request failed with {e}."
                        );
                        request_state = ServiceRequestState::Failed.to_string();
                    }
                };
            }
            state_mgmt
                .send(StateManagement::Set((
                    format!("{robot_name}_dashboard_request_state"),
                    request_state.to_spvalue(),
                )))
                .await?;
        }
        timer.tick().await?;
    }
}
