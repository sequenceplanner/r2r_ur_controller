use std::error::Error;
use tokio::process::Command;
use tokio::task::JoinHandle;

pub async fn robot_state_publisher(urdf: &str) -> Result<JoinHandle<()>, Box<dyn Error>> {
    // Copy the URDF string so we can move it into the spawned task
    let urdf_owned = urdf.to_string();

    // This async block will run in the spawned task
    let child_future = async move {
        // Prepare the child process:
        // Example: ros2 run robot_state_publisher robot_state_publisher robot_description:=<URDF>
        let mut child = Command::new("ros2")
            .arg("run")
            .arg("robot_state_publisher")
            .arg("robot_state_publisher")
            .arg("--ros-args")
            .arg("-p")
            .arg(format!("robot_description:={}", urdf_owned))
            .spawn()
            .expect("Failed to spawn robot_state_publisher process");

        // Wait for the process to complete
        match child.wait().await {
            Ok(status) => {
                if !status.success() {
                    eprintln!("robot_state_publisher exited with code {:?}", status.code());
                } else {
                    println!("robot_state_publisher exited successfully");
                }
            }
            Err(e) => {
                eprintln!("Error waiting for child: {e}");
            }
        }
    };

    // Spawn the background task
    let handle = tokio::spawn(child_future);

    // Return the handle
    Ok(handle)
}