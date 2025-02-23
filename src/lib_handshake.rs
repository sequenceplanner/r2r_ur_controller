use std::net::SocketAddr;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use futures::{SinkExt, StreamExt};
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{mpsc, oneshot, watch};
use tokio::time::timeout;
use tokio_util::codec::{Framed, LinesCodec};

/// The same driver state you want:
pub struct DriverState {
    pub running: bool,
    pub connected: bool,
    pub goal_id: Option<String>,
    pub goal_sender: Option<oneshot::Sender<bool>>,
    pub handshake_sender: Option<oneshot::Sender<bool>>,
    pub feedback_sender: Option<mpsc::Sender<String>>,
    pub robot_state: i32,
    pub program_state: i32,
    pub joint_values: Vec<f64>,
    pub joint_speeds: Vec<f64>,
    pub input_bit0: bool,
    pub input_bit1: bool,
    pub input_bit2: bool,
    pub input_bit3: bool,
    pub input_bit4: bool,
    pub input_bit5: bool,
    pub input_bit6: bool,
    pub input_bit7: bool,
    pub input_bit8: bool,
    pub input_bit9: bool,
    pub output_bit0: bool,
    pub output_bit1: bool,
    pub output_bit2: bool,
    pub output_bit3: bool,
    pub output_bit4: bool,
    pub output_bit5: bool,
    pub output_bit6: bool,
    pub output_bit7: bool,
}

impl DriverState {
    pub fn new() -> Self {
        DriverState {
            running: true,
            connected: false,
            goal_id: None,
            goal_sender: None,
            handshake_sender: None,
            feedback_sender: None,
            robot_state: 0,
            program_state: 0,
            joint_values: vec![],
            joint_speeds: vec![],
            input_bit0: false,
            input_bit1: false,
            input_bit2: false,
            input_bit3: false,
            input_bit4: false,
            input_bit5: false,
            input_bit6: false,
            input_bit7: false,
            input_bit8: false,
            input_bit9: false,
            output_bit0: false,
            output_bit1: false,
            output_bit2: false,
            output_bit3: false,
            output_bit4: false,
            output_bit5: false,
            output_bit6: false,
            output_bit7: false,
        }
    }
}

/// A simple enum for “dashboard” commands to send to the robot:
#[derive(Clone, PartialEq, Debug)]
pub enum DashboardCommand {
    Stop,
    ResetProtectiveStop,
}

/// The item we send to the dashboard task: (command, oneshot_for_result).
pub type DashboardChannelItem = (DashboardCommand, oneshot::Sender<bool>);

/// A “script” request we can send:
/// - `goal_id` is any ID (UUID, etc.)
/// - `script` is the URScript code
/// - `result_tx` gets the final success/failure
/// - `feedback_tx` gets any textual feedback from the script
pub struct ScriptRequest {
    pub goal_id: String,
    pub script: String,
    pub result_tx: oneshot::Sender<bool>,
    pub feedback_tx: mpsc::Sender<String>,
}

/// Utility to read big-endian f64 from 8 bytes
fn read_f64(slice: &[u8]) -> f64 {
    let mut bytes = [0u8; 8];
    bytes.copy_from_slice(slice);
    f64::from_be_bytes(bytes)
}

/// Connect loop keeps trying to connect to address until it succeeds.
async fn connect_loop(address: &str) -> TcpStream {
    loop {
        let ret = TcpStream::connect(address).await;
        match ret {
            Ok(s) => {
                let local_address = s.local_addr().expect("could net get local address");
                let peer_address = s.peer_addr().expect("could net get local address");
                println!(
                    "connected to: {} with host ip {}",
                    peer_address, local_address
                );
                return s;
            }
            Err(e) => {
                println!("could not connect to realtime at {}: {}", address, e);
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

/// Takes the original UR script to call and wraps it in handshaking
/// using socket communication to the host (caller).
fn generate_ur_script(original: &str, host_address: &str) -> String {
    let indented_script: String = original
        .lines()
        .map(|l| format!("  {l}"))
        .collect::<Vec<String>>()
        .join("\n");

    let pre_script = r#"
def run_script():
"#
    .to_string();

    let post_script_1 = r#"

  def handshake():
"#
    .to_string();

    let post_script_2 = format!(
        "    socket_open(\"{}\", 50000, \"ur_driver_socket\")",
        host_address
    );

    let post_script_3 = r#"
    line_from_server = socket_read_line("ur_driver_socket", timeout=1.0)
    if(str_empty(line_from_server)):
      return False
    else:
      socket_send_line(line_from_server, "ur_driver_socket")
      return True
    end
  end

  if(handshake()):
    result = script()
    if(result):
      socket_send_line("ok", "ur_driver_socket")
    else:
      socket_send_line("error", "ur_driver_socket")
    end
  else:
"#
    .to_string();
    let post_script_4 = format!(
        "    popup(\"handshake failure with host {}, not moving.\")",
        host_address
    );

    let post_script_5 = r#"
  end

  socket_close("ur_driver_socket")
end

run_script()
"#
    .to_string();

    return format!(
        "{}{}{}{}{}{}{}",
        pre_script,
        indented_script,
        post_script_1,
        post_script_2,
        post_script_3,
        post_script_4,
        post_script_5
    );
}

// /// Generate a URScript that wraps user code with handshake logic to <host_address>:50000.
// fn generate_ur_script(original: &str, host_address: &str) -> String {
//     let indented_script: String = original
//         .lines()
//         .map(|line| format!("  {line}"))
//         .collect::<Vec<String>>()
//         .join("\n");

//     let pre_script = r#"
// def run_script():
// "#;

//     let post_script = format!(
//         r#"
//   def handshake():
//     socket_open("{host_address}", 50000, "ur_driver_socket")
//     line_from_server = socket_read_line("ur_driver_socket", timeout=1.0)
//     if(str_empty(line_from_server)):
//       return False
//     else:
//       socket_send_line(line_from_server, "ur_driver_socket")
//       return True
//     end
//   end

//   if(handshake()):
//     result = script()
//     if(result):
//       socket_send_line("ok", "ur_driver_socket")
//     else:
//       socket_send_line("error", "ur_driver_socket")
//     end
//   else:
//     popup("Handshake failure with host {host_address}, not moving.")
//   end

//   socket_close("ur_driver_socket")
// end

// run_script()
// "#
//     );

//     [pre_script, &indented_script, &post_script].join("\n")
// }

/// The server on port 50000 to receive handshake from the robot’s URScript:
async fn socket_server(
    ds: Arc<Mutex<DriverState>>,
    // dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
    mut local_addr: watch::Receiver<Option<SocketAddr>>,
) -> Result<(), Box<dyn std::error::Error>> {
    // let bind_addr = SocketAddr::new([0, 0, 0, 0].into(), 50000);
    // println!("Starting handshake socket server at {bind_addr}");

    // let listener = TcpListener::bind(bind_addr).await?;

    let mut addr = None;
    while addr.is_none() {
        local_addr.changed().await?;
        addr = local_addr.borrow().clone();
    }

    let mut addr = addr.unwrap();
    addr.set_port(50000);

    println!("Starting socket server at {}", addr);

    let listener = TcpListener::bind(&addr).await?;

    loop {
        let (stream, addr) = listener.accept().await?;
        println!("New URScript connection from {addr}");

        // Retrieve goal_id + handshake channel + feedback channel from ds
        let (goal_id, handshake_tx, feedback_tx) = {
            let mut state = ds.lock().unwrap();
            if state.handshake_sender.is_none()
                || state.goal_id.is_none()
                || state.feedback_sender.is_none()
            {
                eprintln!("No active handshake/goal, ignoring new connection.");
                continue;
            }
            let hstx = state.handshake_sender.take().unwrap();
            let id = state.goal_id.clone().unwrap();
            let ftx = state.feedback_sender.clone().unwrap();
            (id, hstx, ftx)
        };

        // line-based protocol
        let mut lines = Framed::new(stream, LinesCodec::new());

        // Send the goal_id to the robot
        if let Err(e) = lines.send(&goal_id).await {
            eprintln!("Failed writing handshake ID: {e}");
            continue;
        }

        // Read back. Must match goal_id or handshake fails.
        match lines.next().await {
            Some(Ok(reply)) if reply == goal_id => {
                println!("Handshake success for goal {goal_id}");
                let _ = handshake_tx.send(true);
            }
            other => {
                eprintln!("Handshake mismatch/failed: {other:?}");
                let _ = handshake_tx.send(false);
            }
        }

        // Now read lines until “ok” or “error” or end:
        while let Some(Ok(line)) = lines.next().await {
            match line.as_str() {
                "ok" => {
                    println!("Script reported OK for {goal_id}");
                    let mut state = ds.lock().unwrap();
                    if let Some(tx) = state.goal_sender.take() {
                        let _ = tx.send(true);
                    }
                    state.feedback_sender = None;
                    state.goal_id = None;
                    break;
                }
                "error" => {
                    println!("Script reported ERROR for {goal_id}");
                    let mut state = ds.lock().unwrap();
                    if let Some(tx) = state.goal_sender.take() {
                        let _ = tx.send(false);
                    }
                    state.feedback_sender = None;
                    state.goal_id = None;
                    break;
                }
                other_line => {
                    println!("Script feedback: {other_line}");
                    if let Err(_) = feedback_tx.send(other_line.to_string()).await {
                        println!("Could not send feedback (receiver gone).");
                    }
                }
            }
        }
    }
}

/// Continuously read from UR’s RTDE port, parse the data, update `DriverState`.
/// Reconnect on timeouts.
async fn realtime_reader(
    ds: Arc<Mutex<DriverState>>,
    rtde_address: String,
    override_host_address: Option<String>,
    local_addr_sender: watch::Sender<Option<SocketAddr>>,
) -> Result<(), Box<dyn std::error::Error>> {
    // let mut stream = connect_loop(&rtde_address).await;
    // {
    //     ds.lock().unwrap().connected = true;
    // }
    // let mut size_buf = [0u8; 4];

    let mut size_bytes = [0u8; 4];

    let mut stream = connect_loop(&rtde_address).await;

    let local_addr = if let Some(s) = &override_host_address {
        SocketAddr::from_str(&format!("{}:0", s))?
    } else {
        stream.local_addr()?
    };
    local_addr_sender.send(Some(local_addr))?;

    ds.lock().unwrap().connected = true;

    loop {
        // read 4 bytes
        let result = timeout(Duration::from_secs(1), stream.read_exact(&mut size_bytes)).await;
        if result.is_err() {
            eprintln!("Timeout reading RTDE data, reconnecting...");
            ds.lock().unwrap().connected = false;
            stream = connect_loop(&rtde_address).await;
            ds.lock().unwrap().connected = true;
            continue;
        }
        let _ = result??; // unwrap the nested error

        let msg_size = u32::from_be_bytes(size_bytes) as usize;
        let mut buf = vec![0u8; msg_size - 4];
        stream.read_exact(&mut buf).await?;

        if msg_size == 1220 {
            // parse example:
            // joint positions at offsets 248..296
            let mut joints = Vec::with_capacity(6);
            for i in 0..6 {
                let idx = 248 + i * 8;
                joints.push(read_f64(&buf[idx..idx + 8]));
            }

            // speeds at offsets 296..344
            let mut speeds = Vec::with_capacity(6);
            for i in 0..6 {
                let idx = 296 + i * 8;
                speeds.push(read_f64(&buf[idx..idx + 8]));
            }

            // digital inputs at offset 680..688
            let digital_inputs = read_f64(&buf[680..688]) as u32;
            // robot_state at 808..816
            let robot_state = read_f64(&buf[808..816]) as i32;
            // digital outputs at 1040..1048
            let digital_outputs = read_f64(&buf[1040..1048]) as u32;
            // program_state at 1048..1056
            let program_state = read_f64(&buf[1048..1056]) as i32;

            {
                let mut st = ds.lock().unwrap();
                st.joint_values = joints;
                st.joint_speeds = speeds;
                st.robot_state = robot_state;
                st.program_state = program_state;

                st.input_bit0 = (digital_inputs & 1) == 1;
                st.input_bit1 = (digital_inputs & 2) == 2;
                st.input_bit2 = (digital_inputs & 4) == 4;
                st.input_bit3 = (digital_inputs & 8) == 8;
                st.input_bit4 = (digital_inputs & 16) == 16;
                st.input_bit5 = (digital_inputs & 32) == 32;
                st.input_bit6 = (digital_inputs & 64) == 64;
                st.input_bit7 = (digital_inputs & 128) == 128;
                st.input_bit8 = (digital_inputs & 65536) == 65536;
                st.input_bit9 = (digital_inputs & 131072) == 131072;

                st.output_bit0 = (digital_outputs & 1) == 1;
                st.output_bit1 = (digital_outputs & 2) == 2;
                st.output_bit2 = (digital_outputs & 4) == 4;
                st.output_bit3 = (digital_outputs & 8) == 8;
                st.output_bit4 = (digital_outputs & 16) == 16;
                st.output_bit5 = (digital_outputs & 32) == 32;
                st.output_bit6 = (digital_outputs & 64) == 64;
                st.output_bit7 = (digital_outputs & 128) == 128;
            }

            // If robot_state != 1 => robot not in normal mode => abort active goal
            if robot_state != 1 {
                let mut st = ds.lock().unwrap();
                if let Some(tx) = st.goal_sender.take() {
                    let _ = tx.send(false);
                }
                st.goal_id = None;
                st.feedback_sender = None;
            }
        } else {
            eprintln!("Unknown RTDE packet size: {msg_size}");
        }
    }
}

/// A small set of commands for the “dashboard” server (port 29999).
pub async fn dashboard_task(
    mut rx: mpsc::Receiver<DashboardChannelItem>,
    dashboard_address: String,
) -> Result<(), Box<dyn std::error::Error>> {
    let stream = connect_loop(&dashboard_address).await;
    let mut stream = BufReader::new(stream);

    // Possibly read a welcome line
    {
        let mut line = String::new();
        let _ = stream.read_line(&mut line).await;
        println!("Dashboard says: {line}");
    }

    // Example: ask for robot model
    {
        stream.write_all(b"get robot model\n").await?;
        stream.flush().await?;
        let mut model = String::new();
        stream.read_line(&mut model).await?;
        println!("Robot model: {model}");
    }

    // Main loop: read commands from channel, send them
    while let Some((cmd, result_tx)) = rx.recv().await {
        let (command_str, expected) = match cmd {
            DashboardCommand::Stop => ("stop\n", "Stopped"),
            DashboardCommand::ResetProtectiveStop => {
                ("unlock protective stop\n", "Protective stop releasing")
            }
        };

        println!("Dashboard command => {command_str}");
        stream.write_all(command_str.as_bytes()).await?;
        stream.flush().await?;

        let mut resp_line = String::new();
        stream.read_line(&mut resp_line).await?;
        if resp_line.contains(expected) {
            let _ = result_tx.send(true);
        } else {
            eprintln!("Dashboard command failed, got: {resp_line}");
            let _ = result_tx.send(false);
        }
    }

    Ok(())
}

/// The script executor: receive script requests, check robot state, send URScript, wait for handshake, etc.
async fn script_executor(
    ds: Arc<Mutex<DriverState>>,
    mut rx: mpsc::Receiver<ScriptRequest>,
    // Possibly we also want the dashboard tx if we want to auto-stop on cancel, etc.
    _dashboard_tx: mpsc::Sender<DashboardChannelItem>,
    ur_address: String,
    local_addr: watch::Receiver<Option<SocketAddr>>,
) -> Result<(), Box<dyn std::error::Error>> {
    while let Some(req) = rx.recv().await {
        println!("Got new script request: goal_id={}", req.goal_id);

        let local_addr = local_addr.borrow().clone();
        if local_addr.is_none() {
            println!(
                "Have not connected to robot yet, rejecting request with goal id: {}, script: '{}'",
                req.goal_id, req.script
            );
            // req.reject().expect("could not reject goal");
            continue;
        }
        // Local addr is not none, safe to unwrap here.
        let local_addr = local_addr.unwrap().ip().to_string();

        // Check if connected + normal mode + no other goal
        {
            let st = ds.lock().unwrap();
            if !st.connected || st.robot_state != 1 {
                eprintln!(
                    "Robot not connected or not in normal mode => fail script {}",
                    req.goal_id
                );
                let _ = req.result_tx.send(false);
                continue;
            }
            if st.goal_id.is_some() {
                eprintln!("Already have an active goal => fail script {}", req.goal_id);
                let _ = req.result_tx.send(false);
                continue;
            }
        }

        // Prepare handshake channel
        let (handshake_tx, handshake_rx) = oneshot::channel::<bool>();
        {
            let mut st = ds.lock().unwrap();
            st.goal_id = Some(req.goal_id.clone());
            st.goal_sender = Some(req.result_tx);
            st.handshake_sender = Some(handshake_tx);
            st.feedback_sender = Some(req.feedback_tx);
        }

        // Connect to UR script port (30003)
        let mut tcp = match TcpStream::connect(&ur_address).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to connect to {ur_address}: {e}");
                let mut st = ds.lock().unwrap();
                if let Some(tx) = st.goal_sender.take() {
                    let _ = tx.send(false);
                }
                st.goal_id = None;
                st.feedback_sender = None;
                continue;
            }
        };

        // Generate handshake-wrapped script
        let full_script = generate_ur_script(&req.script, &local_addr);
        // let full_script = generate_ur_script(&req.script, "192.168.1.195");
        println!("Sending script:\n{full_script}");

        if let Err(e) = tcp.write_all(full_script.as_bytes()).await {
            eprintln!("Failed sending script: {e}");
            let mut st = ds.lock().unwrap();
            if let Some(tx) = st.goal_sender.take() {
                let _ = tx.send(false);
            }
            st.goal_id = None;
            st.feedback_sender = None;
            continue;
        }
        let _ = tcp.flush().await;

        // Wait up to 1 second for handshake result:
        match tokio::time::timeout(Duration::from_secs(3), handshake_rx).await {
            Err(_) => {
                eprintln!("Handshake timed out: {}", req.goal_id);
                let mut st = ds.lock().unwrap();
                if let Some(tx) = st.goal_sender.take() {
                    let _ = tx.send(false);
                }
                st.goal_id = None;
                st.feedback_sender = None;
            }
            Ok(Ok(true)) => {
                println!("Handshake ok => script running: {}", req.goal_id);
                // Final success/failure will come from socket_server reading “ok/error”.
            }
            _ => {
                eprintln!("Handshake failed: {}", req.goal_id);
                let mut st = ds.lock().unwrap();
                if let Some(tx) = st.goal_sender.take() {
                    let _ = tx.send(false);
                }
                st.goal_id = None;
                st.feedback_sender = None;
            }
        }
    }

    Ok(())
}

/// We bundle the driver’s key “senders” (channels) so we can hand them back to the caller.
#[derive(Clone)]
pub struct DriverHandles {
    pub script_tx: mpsc::Sender<ScriptRequest>,
    pub dashboard_tx: mpsc::Sender<DashboardChannelItem>,
}

/// Start the driver tasks in the background, returning both:
/// - DriverHandles (the MPSC senders)
/// - Arc<Mutex<DriverState>> so you can read the robot state externally
pub async fn start_driver(
    ur_ip: &str,
    override_host_address: Option<String>,
) -> Result<(DriverHandles, Arc<Mutex<DriverState>>), Box<dyn std::error::Error>> {
    let driver_state = Arc::new(Mutex::new(DriverState::new()));

    // Build addresses
    let rtde_address = format!("{}:30003", ur_ip);
    let dashboard_address = format!("{}:29999", ur_ip);

    // Create channels
    let (dashboard_tx, dashboard_rx) = mpsc::channel::<DashboardChannelItem>(10);
    let (script_tx, script_rx) = mpsc::channel::<ScriptRequest>(10);
    let (local_addr_sender, local_addr_receiver) = watch::channel(None);

    // Spawn tasks exactly as before:
    {
        let ds_clone = driver_state.clone();
        let rtde = rtde_address.clone();
        tokio::spawn(async move {
            if let Err(e) =
                realtime_reader(ds_clone, rtde, override_host_address, local_addr_sender).await
            {
                eprintln!("realtime_reader stopped: {e}");
            }
        });
    }
    {
        let ds_clone = driver_state.clone();
        let local_addr_receiver_clone = local_addr_receiver.clone();
        tokio::spawn(async move {
            if let Err(e) = socket_server(ds_clone, local_addr_receiver_clone).await {
                eprintln!("socket_server stopped: {e}");
            }
        });
    }
    {
        tokio::spawn(async move {
            if let Err(e) = dashboard_task(dashboard_rx, dashboard_address).await {
                eprintln!("dashboard_task stopped: {e}");
            }
        });
    }
    {
        let ds_clone = driver_state.clone();
        let dash_clone = dashboard_tx.clone();
        let local_addr_receiver_clone = local_addr_receiver.clone();
        tokio::spawn(async move {
            if let Err(e) = script_executor(
                ds_clone,
                script_rx,
                dash_clone,
                rtde_address,
                local_addr_receiver_clone,
            )
            .await
            {
                eprintln!("script_executor stopped: {e}");
            }
        });
    }

    // Return the handles + the driver_state itself
    Ok((
        DriverHandles {
            script_tx,
            dashboard_tx,
        },
        driver_state,
    ))
}
