// use core::structs::RobotCommand;
// use std::io::{Error, ErrorKind};
// use std::net::SocketAddr;
// use std::process::Command;
// use std::sync::{Arc, Mutex};
// use std::time::{Duration, Instant};

// use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
// use tokio::net::TcpStream;
// use tokio::sync::{mpsc, oneshot};
// use tokio::time::timeout;



pub mod core;
pub use core::driver::*;
pub use core::structs::*;

pub mod ros;
pub use ros::action_client::*;

// // pub
// // use r2r_ur_controller::*;

// // A high-level controller “wrapper” around your driver code.
// #[derive(Debug, Clone)]
// pub struct URControllerServer {
//     driver_state: Arc<Mutex<DriverState>>,
//     handles: DriverHandles,
//     templates: tera::Tera
// }

// impl URControllerServer {
//     /// Connect to the robot (or simulator) at the given `ip`, spawn all driver tasks.
//     pub async fn connect(ip: &str) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
//         let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");
//         let templates_path = format!("{}", manifest_dir);
//         let templates: tera::Tera = {
//             let tera = match tera::Tera::new(&format!("{}/templates/*.script", templates_path))
//             {
//                 Ok(t) => {
//                     println!("Searching for Tera templates, wait...",);
//                     t
//                 }
//                 Err(e) => {
//                     println!("UR Script template parsing error(s): {}", e);
//                     ::std::process::exit(1);
//                 }
//             };
//             println!("Loaded templates:");
//             for temp in tera.get_template_names() {
//                 println!("{temp}")
//             }
//             tera
//         };

//         let (handles, state) = start_driver(ip).await?;
//         Ok(Self {
//             driver_state: state,
//             handles,
//             templates
//         })
//     }

//     pub async fn execute(
//         &self,
//         command: RobotCommand,
//     ) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
//         let script = generate_script(command, &self.templates).await.expect("msg");
//         // println!("{}", script);
//         // We do the usual oneshot for the result:
//         let (result_tx, result_rx) = oneshot::channel();

//         // Send the script request:
//         self.handles
//             .script_tx
//             .send(ScriptRequest { script, result_tx })
//             .await
//             .map_err(|_| "Failed to send script request")?;

//         // Wait for final success/failure from the driver:
//         let success = result_rx.await?;
//         Ok(success)
//     }

//     // /// Example: movej with 6 joint positions.
//     // /// Returns `Ok(true)` if the script finished successfully, `Ok(false)` if aborted, or `Err(...)` on I/O errors.
//     pub async fn movej(
//         &self,
//         joint_positions: [f64; 6],
//     ) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
//         // Build a minimal URScript
//         // NOTE: you can adjust speed, acceleration, etc. as needed.
//         let script = format!("def my_script():\n  movej({:?})\nend\n", joint_positions);

//         // We do the usual oneshot for the result:
//         let (result_tx, result_rx) = oneshot::channel();

//         // Send the script request:
//         self.handles
//             .script_tx
//             .send(ScriptRequest { script, result_tx })
//             .await
//             .map_err(|_| "Failed to send script request")?;

//         // Wait for final success/failure from the driver:
//         let success = result_rx.await?;
//         Ok(success)
//     }

//     // /// Example: movel with 6 pose arguments
//     // pub async fn movel(
//     //     &self,
//     //     pose: [f64; 6],
//     // ) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
//     //     let script = format!("def my_script():\n  movel({:?})\nend\n", pose);

//     //     let (result_tx, result_rx) = oneshot::channel();
//     //     self.handles
//     //         .script_tx
//     //         .send(ScriptRequest { script, result_tx })
//     //         .await
//     //         .map_err(|_| "Failed to send script request")?;

//     //     let success = result_rx.await?;
//     //     Ok(success)
//     // }

//     /// Example: send a dashboard “Stop” command, returning success/failure
//     pub async fn stop(&self) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
//         let (resp_tx, resp_rx) = oneshot::channel();
//         self.handles
//             .dashboard_tx
//             .send((DashboardCommand::Stop, resp_tx))
//             .await
//             .map_err(|_| "Failed to send dashboard Stop")?;
//         let ok = resp_rx.await?;
//         Ok(ok)
//     }

//     /// Example: reset protective stop
//     pub async fn reset_protective_stop(
//         &self,
//     ) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
//         let (resp_tx, resp_rx) = oneshot::channel();
//         self.handles
//             .dashboard_tx
//             .send((DashboardCommand::ResetProtectiveStop, resp_tx))
//             .await
//             .map_err(|_| "Failed to send dashboard reset")?;
//         let ok = resp_rx.await?;
//         Ok(ok)
//     }

//     /// Example: read current robot_state
//     pub fn get_robot_state(&self) -> i32 { // make a struct to know
//         let ds = self.driver_state.lock().unwrap();
//         ds.robot_state
//     }

//     // etc... add more convenience methods if desired
// }