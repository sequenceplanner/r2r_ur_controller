use std::error::Error;
use tokio::process::Command;
use tokio::task::JoinHandle;

pub async fn ur_script_driver(ur_address: Option<String>, override_host_address: Option<String>) -> Result<JoinHandle<()>, Box<dyn Error>> {
    let child_future = async move {
        let mut child = Command::new("ros2")
            .arg("run")
            .arg("ur_script_driver")
            .arg("ur_script_driver")
            .arg("--ros-args")
            .arg("-p")
            .arg(match ur_address {
                Some(ur_addr) => format!("ur_address:={}", ur_addr),
                None => "".to_string()

            })
            .arg(match override_host_address {
                Some(override_host_addr) => format!("override_host_address:={}", override_host_addr),
                None => "".to_string()

            })
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