use std::io::{Error, ErrorKind};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::timeout;

#[derive(Debug)]
pub struct DriverState {
    pub running: bool,
    pub connected: bool,
    // only handle one "goal" (script) at a time, reply with true/false if script executed successfully.
    pub goal_sender: Option<oneshot::Sender<bool>>,
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
    fn new() -> Self {
        DriverState {
            running: true,
            connected: false,
            goal_sender: None,
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

#[derive(Clone, Debug, PartialEq)]
pub enum DashboardCommand {
    Stop,
    ResetProtectiveStop,
}

/// The command we can send to the script_executor to run a URScript
pub struct ScriptRequest {
    pub script: String,
    pub result_tx: oneshot::Sender<bool>,
    // pub cancel_rx: oneshot::Receiver<()>,
}

fn read_f64(slice: &[u8]) -> f64 {
    let mut bytes = [0u8; 8];
    bytes.copy_from_slice(slice);
    f64::from_be_bytes(bytes)
}

async fn connect_loop(address: &str) -> TcpStream {
    loop {
        match TcpStream::connect(address).await {
            Ok(s) => {
                println!("Connected to {address}");
                return s;
            }
            Err(e) => {
                eprintln!("Could not connect to {address}: {e}, retrying in 1 second...");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

async fn realtime_reader(
    driver_state: Arc<Mutex<DriverState>>,
    ur_address: String,
) -> Result<(), Error> {
    let mut size_bytes = [0u8; 4];

    let mut checking_for_program_to_run = false;
    let mut waiting_for_program_running_since: Option<Instant> = None;

    // Connect once, keep reconnecting on timeouts
    let mut stream = connect_loop(&ur_address).await;
    {
        driver_state.lock().unwrap().connected = true;
    }

    loop {
        // read 4 bytes with 1 sec timeout
        let ret = timeout(Duration::from_secs(1), stream.read_exact(&mut size_bytes)).await;
        if let Err(_) = ret {
            // timed out => reconnect
            {
                let mut ds = driver_state.lock().unwrap();
                ds.connected = false;
                // if we had an active goal => abort
                if let Some(tx) = ds.goal_sender.take() {
                    eprintln!("Timeout => aborting active script");
                    let _ = tx.send(false);
                }
            }

            checking_for_program_to_run = false;
            waiting_for_program_running_since = None;

            println!("Timeout on read => reconnecting...");
            stream = connect_loop(&ur_address).await;
            driver_state.lock().unwrap().connected = true;
            continue;
        }

        // unwrap the inner result
        let _ = ret??;

        // parse msg_size
        let msg_size = u32::from_be_bytes(size_bytes) as usize;
        let mut buf = vec![0u8; msg_size - 4];
        stream.read_exact(&mut buf).await?;

        // UR usually sends 1220 bytes for the real-time state
        if msg_size != 1220 {
            eprintln!("Got unknown frame length: {msg_size}");
            continue;
        }

        // Parse joint angles (example offsets)
        let mut joints = vec![];
        for i in 0..6 {
            let idx = 248 + i * 8;
            joints.push(read_f64(&buf[idx..idx + 8]));
        }
        // Parse joint speeds
        let mut speeds = vec![];
        for i in 0..6 {
            let idx = 296 + i * 8;
            speeds.push(read_f64(&buf[idx..idx + 8]));
        }
        // digital inputs at offset 680..688
        let digital_inputs = read_f64(&buf[680..688]) as u32;
        // robot_state at offset 808..816
        let robot_state = read_f64(&buf[808..816]) as i32;
        // digital outputs at offset 1040..1048
        let digital_outputs = read_f64(&buf[1040..1048]) as u32;
        // program_state at offset 1048..1056
        let program_state = read_f64(&buf[1048..1056]) as i32;

        // Update driver_state
        let (goal_active, _old_program_state) = {
            let mut ds = driver_state.lock().unwrap();
            ds.joint_values = joints;
            ds.joint_speeds = speeds;
            ds.robot_state = robot_state;
            ds.program_state = program_state;

            // set bits
            ds.input_bit0 = (digital_inputs & 1) == 1;
            ds.input_bit1 = (digital_inputs & 2) == 2;
            ds.input_bit2 = (digital_inputs & 4) == 4;
            ds.input_bit3 = (digital_inputs & 8) == 8;
            ds.input_bit4 = (digital_inputs & 16) == 16;
            ds.input_bit5 = (digital_inputs & 32) == 32;
            ds.input_bit6 = (digital_inputs & 64) == 64;
            ds.input_bit7 = (digital_inputs & 128) == 128;
            ds.input_bit8 = (digital_inputs & 65536) == 65536;
            ds.input_bit9 = (digital_inputs & 131072) == 131072;

            ds.output_bit0 = (digital_outputs & 1) == 1;
            ds.output_bit1 = (digital_outputs & 2) == 2;
            ds.output_bit2 = (digital_outputs & 4) == 4;
            ds.output_bit3 = (digital_outputs & 8) == 8;
            ds.output_bit4 = (digital_outputs & 16) == 16;
            ds.output_bit5 = (digital_outputs & 32) == 32;
            ds.output_bit6 = (digital_outputs & 64) == 64;
            ds.output_bit7 = (digital_outputs & 128) == 128;

            (ds.goal_sender.is_some(), ds.program_state)
        };

        // If robot_state != 1 => protective/emergency => abort
        if robot_state != 1 && goal_active {
            let mut ds = driver_state.lock().unwrap();
            if let Some(tx) = ds.goal_sender.take() {
                eprintln!("Protective stop => abort script");
                let _ = tx.send(false);
            }
            checking_for_program_to_run = false;
            waiting_for_program_running_since = None;
            continue;
        }

        // We track program start => program_state=2 => then wait for program_state=1 => finished

        // If there's a goal, we check for program_state transitions:
        if goal_active {
            // If we haven't yet seen program_state=2, we wait for it
            if !checking_for_program_to_run && program_state == 2 {
                println!("Program started => now waiting for it to finish");
                checking_for_program_to_run = true;
            }
            // If we haven't seen "2" yet, but it's 1, we might time out
            if !checking_for_program_to_run && program_state == 1 {
                // we start a small timer for program_state=2
                if waiting_for_program_running_since.is_none() {
                    waiting_for_program_running_since = Some(Instant::now());
                } else {
                    let elapsed = waiting_for_program_running_since.unwrap().elapsed();
                    if elapsed > Duration::from_secs(1) {
                        // if after 1 sec, never reached program_state=2 => fail
                        println!("Never reached program running state => abort script");
                        waiting_for_program_running_since = None;
                        let mut ds = driver_state.lock().unwrap();
                        if let Some(tx) = ds.goal_sender.take() {
                            let _ = tx.send(false);
                        }
                    }
                }
            }
            // If we have seen "2" and now it's back to "1", that means done
            if checking_for_program_to_run && program_state == 1 {
                println!("Program finished => success");
                checking_for_program_to_run = false;
                waiting_for_program_running_since = None;
                let mut ds = driver_state.lock().unwrap();
                if let Some(tx) = ds.goal_sender.take() {
                    let _ = tx.send(true);
                }
            }
        }
    }
}


async fn dashboard(
    mut recv: mpsc::Receiver<(DashboardCommand, oneshot::Sender<bool>)>,
    ur_address: String,
) -> Result<(), Error> {
    let stream = connect_loop(&ur_address).await;
    let mut stream = BufReader::new(stream);

    // read welcome line
    let mut line = String::new();
    stream.read_line(&mut line).await?;
    if !line.contains("Connected: Universal Robots Dashboard Server") {
        return Err(Error::new(ErrorKind::Other, "Not a UR Dashboard?"));
    }
    // Example: ask for robot model
    stream.write_all(b"get robot model\n").await?;
    stream.flush().await?;
    let mut robot_model = String::new();
    stream.read_line(&mut robot_model).await?;
    println!("robot model: {robot_model}");

    // main loop
    while let Some((command, resp_tx)) = recv.recv().await {
        println!("Got dashboard command: {:?}", command);
        let (cmd_str, expected_substring) = match command {
            DashboardCommand::Stop => ("stop\n", "Stopped"),
            DashboardCommand::ResetProtectiveStop => ("unlock protective stop\n", "Protective stop releasing"),
        };

        stream.write_all(cmd_str.as_bytes()).await?;
        stream.flush().await?;

        let mut response = String::new();
        stream.read_line(&mut response).await?;
        if response.contains(expected_substring) {
            let _ = resp_tx.send(true);
        } else {
            eprintln!("Dashboard cmd failed. Got: {response}");
            let _ = resp_tx.send(false);
        }
    }

    Ok(())
}

// receives script requests, writes them to port 30003
async fn script_executor(
    ds: Arc<Mutex<DriverState>>,
    mut rx: mpsc::Receiver<ScriptRequest>,
    ur_address: String,
    // dashboard_tx: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
) -> Result<(), Error> {
    while let Some(request) = rx.recv().await {
        println!("Got script => {}", request.script);

        // Check if robot is connected & normal mode
        {
            let st = ds.lock().unwrap();
            if !st.connected { // } || st.robot_state != 1 {
                eprintln!("Robot not connected or not normal => fail script immediately");
                let _ = request.result_tx.send(false);
                continue;
            }
            if st.goal_sender.is_some() {
                eprintln!("Already have an active script => fail script immediately");
                let _ = request.result_tx.send(false);
                continue;
            }
        }

        // Insert new goal_sender
        {
            let mut st = ds.lock().unwrap();
            st.goal_sender = Some(request.result_tx);
        }

        // Connect to 30003 to write the script
        let mut tcp = match TcpStream::connect(&ur_address).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to connect to {ur_address}: {e}");
                let mut st = ds.lock().unwrap();
                if let Some(tx) = st.goal_sender.take() {
                    let _ = tx.send(false);
                }
                continue;
            }
        };
        tcp.write_all(request.script.as_bytes()).await?;
        tcp.flush().await?;

        // Now we do NOT block here for final result. The real-time reader
        // determines when the program finishes => triggers `goal_sender.send(true/false)`.
        // To cancel do something like:
        //   let (cancel_tx, cancel_rx) = oneshot::channel();
        //   dashboard_tx.send((DashboardCommand::Stop, cancel_tx)).await?...
    }

    Ok(())
}



/// Channels to interact with the driver: 
/// - `script_tx` to send URScripts 
/// - `dashboard_tx` to send Stop/ResetProtectiveStop commands
#[derive(Debug, Clone)]
pub struct DriverHandles {
    pub script_tx: mpsc::Sender<ScriptRequest>,
    pub dashboard_tx: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
}

/// Start the driver tasks in the background, returning:
///  - (DriverHandles, Arc<Mutex<DriverState>>)
/// so the caller can send scripts, send dashboard commands, and observe the driver state.
pub async fn start_driver(
    ip: &str,  // e.g. "192.168.0.10" or "127.0.0.1" for sim
) -> Result<(DriverHandles, Arc<Mutex<DriverState>>), Box<dyn std::error::Error + Send + Sync>> {
    let ds = Arc::new(Mutex::new(DriverState::new()));

    let (script_tx, script_rx) = mpsc::channel::<ScriptRequest>(10);
    let (dash_tx, dash_rx) = mpsc::channel::<(DashboardCommand, oneshot::Sender<bool>)>(10);

    let rtde_addr = format!("{ip}:30003");
    let dash_addr = format!("{ip}:29999");

    // Spawn realtime_reader
    {
        let ds_clone = ds.clone();
        let rtde_addr_clone = rtde_addr.clone();
        tokio::spawn(async move {
            if let Err(e) = realtime_reader(ds_clone, rtde_addr_clone).await {
                eprintln!("realtime_reader ended: {e}");
            }
        });
    }

    // Spawn dashboard
    {
        tokio::spawn(async move {
            if let Err(e) = dashboard(dash_rx, dash_addr).await {
                eprintln!("dashboard ended: {e}");
            }
        });
    }

    // Spawn script_executor
    {
        let ds_clone = ds.clone();
        let rtde_clone = rtde_addr;
        // let dash_clone = dash_tx.clone();
        tokio::spawn(async move {
            if let Err(e) = script_executor(ds_clone, script_rx, rtde_clone).await {
                eprintln!("script_executor ended: {e}");
            }
        });
    }

    // Return
    let handles = DriverHandles {
        script_tx,
        dashboard_tx: dash_tx,
    };
    Ok((handles, ds))
}