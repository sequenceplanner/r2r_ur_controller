// use std::io::{Error, ErrorKind};
// use std::net::SocketAddr;
// use std::sync::{Arc, Mutex};
// use std::time::{Duration, Instant};

// use r2r_transforms::TransformStamped;
// use r2r_ur_controller::core::structs::{pose_to_string, Payload, RobotCommand};
// use r2r_ur_controller::*;
// use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
// use tokio::net::TcpStream;
// use tokio::sync::{mpsc, oneshot};
// use tokio::time::timeout;

// // #[tokio::main]
// // async fn main() -> Result<(), Box<dyn std::error::Error>> {
// //     // 1) Connect to the UR driver
// //     let driver = URControllerServer::connect("127.0.0.1").await.expect("msg");

// //     // 2) Wait for the driver to show “connected” if you want:
// //     loop {
// //         let state = driver.get_robot_state();
// //         if state == 1 {
// //             println!("Robot is in normal mode => can proceed");
// //             break;
// //         }
// //         tokio::time::sleep(std::time::Duration::from_millis(100)).await;
// //     }

// //     // 3) Send a movej command
// //     let success = driver.movej([0.0, 0.0, 0.0, -1.57, 0.0, 0.0]).await.expect("msg");
// //     println!("movej => success? {}", success);

// //     // 4) Possibly stop the robot
// //     let stop_ok = driver.stop().await.expect("msg");
// //     println!("Stop => success? {}", stop_ok);

// //     Ok(())
// // }

// #[tokio::main]
// async fn main() -> Result<(), Box<dyn std::error::Error>> {
//     // 1) Create the driver
//     let driver = URControllerServer::connect("127.0.0.1").await.expect("msg");

//     // 2) Wait for normal mode (robot_state == 1), optional
//     loop {
//         if driver.get_robot_state() == 1 {
//             println!("Robot is in normal mode => can proceed.");
//             break;
//         }
//         tokio::time::sleep(Duration::from_millis(100)).await;
//     }

//     // let tcp_in_faceplate = match message.use_joint_positions && message.command.contains("move_j") {
//     //     true => pose_to_string(&TransformStamped::default()),
//     //     false => match lookup_tf(FACEPLATE_ID, &message.tcp_id, tf_lookup_client).await {
//     //         Some(transform) => pose_to_string(&transform),
//     //         None => return None,
//     //     },
//     // };

//     // 3) Spawn a task that does movej
//     let driver_arc = Arc::new(Mutex::new(driver.clone()));
//     // let command = RobotCommand {
//     //     command: core::structs::CommandType::MoveJ,
//     //     acceleration: 0.5,
//     //     velocity: 0.5,
//     //     global_acceleration_scaling: 1.0,
//     //     global_velocity_scaling: 1.0,
//     //     use_execution_time: false,
//     //     execution_time: 0.0,
//     //     use_blend_radius: false,
//     //     blend_radius: 0.0,
//     //     use_joint_positions: true,
//     //     joint_positions: vec![-1.57, 0.0, 0.0, -1.57, 0.0, 0.0],
//     //     use_preferred_joint_config: false,
//     //     preferred_joint_config: vec![],
//     //     use_payload: false,
//     //     payload: "".to_string(),
//     //     baseframe_id: "".to_string(),
//     //     faceplate_id: "".to_string(),
//     //     goal_feature_id: "".to_string(),
//     //     tcp_id: "".to_string(),
//     //     target_in_base: "".to_string(),
//     //     set_tcp: false,
//     //     tcp_in_faceplate: pose_to_string(&TransformStamped::default()),
//     // };

//     // let driver_for_move = driver_arc.clone();
//     // let h = tokio::spawn(async move {
//     //     let drv = driver_for_move.lock().unwrap().clone();
//     //     drv.execute(command).await
//     // });

//     // let driver_for_move = driver_arc.clone();
//     // match tokio::spawn(async move {
//     //     // This will block here until script finishes or is stopped
//     //     let drv = driver_for_move.lock().unwrap().clone();
//     //     drv.execute(command).await
//     //     // Ok::<(), Box<dyn std::error::Error>>(())
//     // })
//     // .await
//     // {
//     //     Ok(Ok(true)) => println!("Move handle completed normally (true)."),
//     //     Ok(Ok(false)) => println!("Move handle completed normally (false)."),
//     //     Ok(Err(e)) => eprintln!("Move handle error: {}", e),
//     //     Err(join_err) => eprintln!("Join error: {}", join_err),
//     // };

//     // // Working
//     // let driver_for_move = driver_arc.clone();
//     // match tokio::spawn(async move {
//     //     // This will block here until script finishes or is stopped
//     //     let drv = driver_for_move.lock().unwrap().clone();
//     //     drv.movej([0.0, 0.0, 0.0, 1.57, 0.0, 0.0]).await
//     // }).await
//     // {
//     //     Ok(Ok(true)) => println!("Move handle completed normally (true)."),
//     //     Ok(Ok(false)) => println!("Move handle completed normally (false)."),
//     //     Ok(Err(e)) => eprintln!("Move handle error: {}", e),
//     //     Err(join_err) => eprintln!("Join error: {}", join_err),
//     // };

//     // 4) Meanwhile, let's wait 2 seconds, then stop the robot (simulating a "cancel")
//     tokio::time::sleep(Duration::from_secs(2)).await;
//     println!("Requesting Stop now...");
//     let stop_ok = driver.stop().await.expect("msg");
//     println!("Stop => success? {}", stop_ok);

//     // 5) The move handle should now return (false) or (true if it finished first).
//     match h.await {
//         Ok(Ok(true)) => println!("Move handle completed normally (true)."),
//         Ok(Ok(false)) => println!("Move handle completed normally (false)."),
//         Ok(Err(e)) => println!("?"),
//         Err(e) => eprintln!("Move handle error: {}", e),
//         // Err(join_err) => eprintln!("Join error: {}", join_err),
//     }

//     Ok(())
// }


#[tokio::main]
async fn main() {}