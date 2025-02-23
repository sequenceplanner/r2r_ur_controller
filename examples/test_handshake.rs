// use r2r_ur_controller::*;
// use serde::ser;
// use std::{
//     sync::{Arc, Mutex},
//     time::Duration,
// };

// use tokio::sync::{mpsc, oneshot};

// // mod driver; // The code above is in driver.rs

// #[tokio::main]
// async fn main() {
//     // Start driver (all tasks) in the background
//     let (handles, driver_state) = start_driver("0.0.0.0", None)
//         .await
//         .expect("Failed to start driver");

//     loop {
//         {
//             let st = driver_state.lock().unwrap();
//             if st.connected && st.robot_state == 1 {
//                 println!("Robot is connected, can now send scripts!");
//                 break;
//             }
//         }
//         // Sleep a short time to avoid busy-looping
//         tokio::time::sleep(std::time::Duration::from_millis(100)).await;
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
//                 tokio::time::sleep(Duration::from_secs(1)).await;
//             }
//         });
//     }

//     // Now you can also send a URScript request (example):
//     let (result_tx, result_rx) = oneshot::channel();
//     let (feedback_tx, mut feedback_rx) = mpsc::channel(10);

//     let script_code = r#"
// def start_vacuum_script():
//   set_tool_digital_out(1, True)
// end
// start_vacuum_script()
// "#;

//     let request = ScriptRequest {
//         goal_id: "my_goal_124".to_string(),
//         script: script_code.to_string(),
//         result_tx,
//         feedback_tx,
//     };

//     handles.script_tx.send(request).await.unwrap();

//     // Optionally spawn a feedback listener
//     tokio::spawn(async move {
//         while let Some(line) = feedback_rx.recv().await {
//             println!("FEEDBACK >> {line}");
//         }
//         println!("Feedback closed.");
//     });

//     // Wait for final success/failure:
//     match result_rx.await {
//         Ok(true) => println!("Script succeeded!"),
//         Ok(false) => println!("Script reported error or was aborted."),
//         Err(e) => println!("Script result channel dropped: {e}"),
//     }

//     // Just keep the program alive for a while
//     tokio::time::sleep(Duration::from_secs(5)).await;
//     println!("Exiting main.");

//     // was working for lib_newer_working
//     // loop {
//     //     match run_driver().await {
//     //         Err(e) => {
//     //             eprintln!("Driver error: {e}. Restarting in 2 seconds...");
//     //             tokio::time::sleep(Duration::from_secs(2)).await;
//     //         }.to_owned(),
//     //         _ => {}
//     //     }

//     //     println!("asdf")
//     // }
// }


#[tokio::main]
async fn main() {}