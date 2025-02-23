use std::io::{Error, ErrorKind};
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use r2r_ur_controller::*;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::timeout;

// #[tokio::main]
// async fn main() -> Result<(), Box<dyn std::error::Error>> {
//     // 1) Connect to the UR driver
//     let driver = URControllerServer::connect("127.0.0.1").await.expect("msg");
    
//     // 2) Wait for the driver to show “connected” if you want:
//     loop {
//         let state = driver.robot_state();
//         if state == 1 {
//             println!("Robot is in normal mode => can proceed");
//             break;
//         }
//         tokio::time::sleep(std::time::Duration::from_millis(100)).await;
//     }

//     // 3) Send a movej command
//     let success = driver.movej([0.0, 0.0, 0.0, -1.57, 0.0, 0.0]).await.expect("msg");
//     println!("movej => success? {}", success);

//     // 4) Possibly stop the robot
//     let stop_ok = driver.stop().await.expect("msg");
//     println!("Stop => success? {}", stop_ok);

//     Ok(())
// }


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1) Create the driver
    let driver = URControllerServer::connect("127.0.0.1").await.expect("msg");

    // 2) Wait for normal mode (robot_state == 1), optional
    loop {
        if driver.robot_state() == 1 {
            println!("Robot is in normal mode => can proceed.");
            break;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // 3) Spawn a task that does movej
    let driver_arc = Arc::new(Mutex::new(driver.clone()));
    // let driver_for_move = driver.clone();
    // let move_handle = 
    tokio::spawn(async move {
        // This will block here until script finishes or is stopped
        let drv = driver_arc.lock().unwrap().clone();
        let success = drv.movej([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]).await.expect("msg");
        println!("movej => success? {}", success);
        // Ok::<(), Box<dyn std::error::Error>>(())
    });

    // 4) Meanwhile, let's wait 2 seconds, then stop the robot (simulating a "cancel")
    tokio::time::sleep(Duration::from_secs(2)).await;
    println!("Requesting Stop now...");
    let stop_ok = driver.stop().await.expect("msg");
    println!("Stop => success? {}", stop_ok);

    // 5) The move handle should now return (false) or (true if it finished first).
    // match move_handle.await {
    //     Ok(Ok(())) => println!("Move handle completed normally."),
    //     Ok(Err(e)) => eprintln!("Move handle error: {}", e),
    //     Err(join_err) => eprintln!("Join error: {}", join_err),
    // }

    Ok(())
}