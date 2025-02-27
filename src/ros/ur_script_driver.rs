use std::error::Error;
use tokio::process::Command;
use tokio::task::JoinHandle;

pub async fn ur_script_driver(override_host_address: Option<String>) -> Result<JoinHandle<()>, Box<dyn Error>> {
    let child_future = async move {
        let mut child = Command::new("ros2")
            .arg("run")
            .arg("ur_script_driver")
            .arg("ur_script_driver")
            .arg("--ros-args")
            .arg("-p")
            .arg(format!("override_host_address:={}", override_host_address.unwrap_or("".to_string())))
            .spawn()
            .expect("Failed to spawn ur_script_driver process");

        match child.wait().await {
            Ok(status) => {
                if !status.success() {
                    eprintln!("ur_script_driver exited with code {:?}", status.code());
                } else {
                    println!("ur_script_driver exited successfully");
                }
            }
            Err(e) => {
                eprintln!("Error waiting for child: {e}");
            }
        }
    };

    let handle = tokio::spawn(child_future);

    Ok(handle)
}