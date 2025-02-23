async fn state_publisher(
    driver_state: Arc<Mutex<DriverState>>,
    joint_publisher: Publisher<sensor_msgs::msg::JointState>,
    measured_publisher: Publisher<ur_script_msgs::msg::Measured>,
    prefix: String
) -> Result<(), Box<dyn std::error::Error>> {
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let joint_names = vec![
        format!("{}shoulder_pan_joint", prefix),
        format!("{}shoulder_lift_joint", prefix),
        format!("{}elbow_joint", prefix),
        format!("{}wrist_1_joint", prefix),
        format!("{}wrist_2_joint", prefix),
        format!("{}wrist_3_joint", prefix),
    ];

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        let now = clock.get_now().unwrap();
        let time = r2r::Clock::to_builtin_time(&now);

        let (joint_values, measured) = {
            let ds = driver_state.lock().unwrap();

            let measured = ur_script_msgs::msg::Measured {
                robot_state: (*ds).robot_state,
                program_state: (*ds).program_state,

                in0: (*ds).input_bit0,
                in1: (*ds).input_bit1,
                in2: (*ds).input_bit2,
                in3: (*ds).input_bit3,
                in4: (*ds).input_bit4,
                in5: (*ds).input_bit5,
                in6: (*ds).input_bit6,
                in7: (*ds).input_bit7,
                in8: (*ds).input_bit8,
                in9: (*ds).input_bit9,

                out0: (*ds).output_bit0,
                out1: (*ds).output_bit1,
                out2: (*ds).output_bit2,
                out3: (*ds).output_bit3,
                out4: (*ds).output_bit4,
                out5: (*ds).output_bit5,
                out6: (*ds).output_bit6,
                out7: (*ds).output_bit7,
            };
            ((*ds).joint_values.clone(), measured)
        };

        let header = std_msgs::msg::Header {
            stamp: time,
            ..Default::default()
        };
        let to_send = sensor_msgs::msg::JointState {
            header,
            position: joint_values,
            name: joint_names.clone(),
            ..Default::default()
        };

        joint_publisher.publish(&to_send).unwrap();
        measured_publisher.publish(&measured).unwrap();
    }
}