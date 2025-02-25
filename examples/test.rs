// use std::io::{Error, ErrorKind};
// use std::net::SocketAddr;
// use std::sync::{Arc, Mutex};
// use std::time::{Duration, Instant};

// use r2r_ur_controller::*;
// use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
// use tokio::net::TcpStream;
// use tokio::sync::{mpsc, oneshot};
// use tokio::time::timeout;

// // #[tokio::main]
// // async fn main() {
// //     // Start everything
// //     let (handles, driver_state) = match start_driver("127.0.0.1").await {
// //         Ok((h, s)) => (h, s),
// //         Err(e) => {
// //             eprintln!("Failed to start driver: {e}");
// //             return;
// //         }
// //     };

// //     // Wait for robot to connect
// //     loop {
// //         let st = driver_state.lock().unwrap();
// //         if st.connected {
// //             println!("Robot is connected => can send scripts now!");
// //             break;
// //         }
// //         drop(st);
// //         tokio::time::sleep(Duration::from_millis(200)).await;
// //     }

// //     // Example: send a script
// //     let (result_tx, result_rx) = oneshot::channel();
// //     let script = "def my_script():\n  movej([0,0,0,0,0,0])\nend\n".to_string();

// //     let (cancel_tx, cancel_rx) = oneshot::channel();

// //     // Build the script request
// //     let req = ScriptRequest {
// //         script,
// //         result_tx,
// //         cancel_rx,
// //     };

// //     // Send the request
// //     handles.script_tx.send(req).await.unwrap();

// //     // Later, if you want to cancel:
// //     // let _ = cancel_tx.send(()); // or just drop it, depending on your design

// //     // handles.script_tx.send(ScriptRequest {
// //     //     script,
// //     //     result_tx,
// //     // }).await.expect("Failed to send script request");

// //     // Wait for final success/failure
// //     match result_rx.await {
// //         Ok(true) => println!("Script execution succeeded"),
// //         Ok(false) => println!("Script execution failed/aborted"),
// //         Err(e) => println!("Script result channel dropped: {e}"),
// //     }

// //     // Example: send a dashboard command
// //     // let (dash_resp_tx, dash_resp_rx) = oneshot::channel();
// //     // handles
// //     //     .dashboard_tx
// //     //     .send((DashboardCommand::Stop, dash_resp_tx))
// //     //     .await
// //     //     .expect("Failed to send dashboard command");
// //     // let dash_ok = dash_resp_rx.await.expect("oneshot canceled?");
// //     // println!("Stop command => success? {dash_ok}");

// //     // // Show some driver state
// //     // {
// //     //     let st = driver_state.lock().unwrap();
// //     //     println!(
// //     //         "Final driver state => robot_state={}, program_state={}",
// //     //         st.robot_state, st.program_state
// //     //     );
// //     // }

// //     // Done
// //     println!("Example main done.");
// // }

// #[tokio::main]
// async fn main() {
//     // Start everything
//     let (handles, driver_state) = match start_driver("127.0.0.1").await {
//         Ok((h, s)) => (h, s),
//         Err(e) => {
//             eprintln!("Failed to start driver: {e}");
//             return;
//         }
//     };

//     // Wait for robot to connect
//     loop {
//         let st = driver_state.lock().unwrap();
//         if st.connected {
//             println!("Robot is connected => can send scripts now!");
//             break;
//         }
//         drop(st);
//         tokio::time::sleep(Duration::from_millis(200)).await;
//     }

//     // Spawn a small task that prints state every second:
//     {
//         let ds = driver_state.clone();
//         tokio::spawn(async move {
//             loop {
//                 // Lock the state to read it
//                 {
//                     let st = ds.lock().unwrap();
//                     println!(
//                         "Robot state={}, Program state={}, Connected={}",
//                         st.robot_state, st.program_state, st.connected
//                     );
//                     println!(
//                         "Inputs: bit0={}, bit1={}, bit2={}, bit3={}, bit4={}, bit5={}, bit6={}, bit7={}, bit8={}, bit9={}",
//                         st.input_bit0,
//                         st.input_bit1,
//                         st.input_bit2,
//                         st.input_bit3,
//                         st.input_bit4,
//                         st.input_bit5,
//                         st.input_bit6,
//                         st.input_bit7,
//                         st.input_bit8,
//                         st.input_bit9
//                     );
//                     println!(
//                         "Outputs: bit0={}, bit1={}, bit2={}, bit3={}, bit4={}, bit5={}, bit6={}, bit7={}",
//                         st.output_bit0,
//                         st.output_bit1,
//                         st.output_bit2,
//                         st.output_bit3,
//                         st.output_bit4,
//                         st.output_bit5,
//                         st.output_bit6,
//                         st.output_bit7
//                     );
//                 }
//                 tokio::time::sleep(Duration::from_millis(100)).await;
//             }
//         });
//     }

//     // Example: send a script
//     let (result_tx, result_rx) = oneshot::channel();
//     let script = "def my_script():\n  movej([0,0,0,0,0,0])\nend\n".to_string();
//     handles
//         .script_tx
//         .send(ScriptRequest { script, result_tx })
//         .await
//         .expect("Failed to send script request");

//     // Wait for final success/failure
//     match result_rx.await {
//         Ok(true) => println!("Script execution succeeded"),
//         Ok(false) => println!("Script execution failed/aborted"),
//         Err(e) => println!("Script result channel dropped: {e}"),
//     }

//     // Example: send a dashboard command
//     let (dash_resp_tx, dash_resp_rx) = oneshot::channel();
//     handles
//         .dashboard_tx
//         .send((DashboardCommand::Stop, dash_resp_tx))
//         .await
//         .expect("Failed to send dashboard command");
//     let dash_ok = dash_resp_rx.await.expect("oneshot canceled?");
//     println!("Stop command => success? {dash_ok}");

//     // Show some driver state
//     {
//         let st = driver_state.lock().unwrap();
//         println!(
//             "Final driver state => robot_state={}, program_state={}",
//             st.robot_state, st.program_state
//         );
//     }

//     // Done
//     println!("Example main done.");
// }

#[tokio::main]
async fn main() {}