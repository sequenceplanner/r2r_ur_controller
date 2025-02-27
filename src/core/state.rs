use micro_sp::*;

pub fn generate_robot_interface_state(robot_name: &str) -> State {
    let state = State::new();

    let request_trigger = bv!(&&format!("{}_request_trigger", robot_name));
    let request_state = v!(&&format!("{}_request_state", robot_name));
    let dashboard_request_trigger = bv!(&&format!("{}_dashboard_request_trigger", robot_name));
    let dashboard_request_state = v!(&&format!("{}_dashboard_request_state", robot_name));
    let total_fail_counter = iv!(&&format!("{}_total_fail_counter", robot_name));
    let subsequent_fail_counter = iv!(&&format!("{}_subsequent_fail_counter", robot_name));

    let state = state.add(assign!(request_trigger, false.to_spvalue()));
    let state = state.add(assign!(request_state, "initial".to_spvalue()));
    let state = state.add(assign!(dashboard_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(dashboard_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(total_fail_counter, 0.to_spvalue()));
    let state = state.add(assign!(subsequent_fail_counter, 0.to_spvalue()));

    let command_type = v!(&&format!("{}_command_type", robot_name));
    let accelleration = fv!(&&format!("{}_accelleration", robot_name));
    let velocity = fv!(&&format!("{}_velocity", robot_name));
    let global_acceleration_scaling = fv!(&&format!("{}_global_acceleration_scaling", robot_name));
    let global_velocity_scaling = fv!(&&format!("{}_global_velocity_scaling", robot_name));
    let use_execution_time = bv!(&&format!("{}_use_execution_time", robot_name));
    let execution_time = fv!(&&format!("{}_execution_time", robot_name));
    let use_blend_radius = bv!(&&format!("{}_use_blend_radius", robot_name));
    let blend_radius = fv!(&&format!("{}_blend_radius", robot_name));
    let use_joint_positions = bv!(&&format!("{}_use_joint_positions", robot_name));
    let joint_positions = av!(&&format!("{}_joint_positions", robot_name));
    let joint_states = av!(&&format!("{}_joint_states", robot_name));
    let use_preferred_joint_config = bv!(&&format!("{}_use_preferred_joint_config", robot_name));
    let preferred_joint_config = av!(&&format!("{}_preferred_joint_config", robot_name));
    let use_payload = bv!(&&format!("{}_use_payload", robot_name));
    let payload = v!(&&format!("{}_payload", robot_name));
    let baseframe_id = v!(&&format!("{}_baseframe_id", robot_name));
    let faceplate_id = v!(&&format!("{}_faceplate_id", robot_name));
    let goal_feature_id = v!(&&format!("{}_goal_feature_id", robot_name));
    let tcp_id = v!(&&format!("{}_tcp_id", robot_name));
    let root_frame_id = v!(&&format!("{}_root_frame_id", robot_name));
    let cancel_current_goal = bv!(&&format!("{}_cancel_current_goal", robot_name));

    let state = state.add(assign!(command_type, SPValue::UNKNOWN));
    let state = state.add(assign!(accelleration, SPValue::UNKNOWN));
    let state = state.add(assign!(velocity, SPValue::UNKNOWN));
    let state = state.add(assign!(global_acceleration_scaling, SPValue::UNKNOWN));
    let state = state.add(assign!(global_velocity_scaling, SPValue::UNKNOWN));
    let state = state.add(assign!(use_execution_time, SPValue::UNKNOWN));
    let state = state.add(assign!(execution_time, SPValue::UNKNOWN));
    let state = state.add(assign!(use_blend_radius, SPValue::UNKNOWN));
    let state = state.add(assign!(blend_radius, SPValue::UNKNOWN));
    let state = state.add(assign!(use_joint_positions, SPValue::UNKNOWN));
    let state = state.add(assign!(joint_positions, SPValue::UNKNOWN));
    let state = state.add(assign!(joint_states, SPValue::UNKNOWN));
    let state = state.add(assign!(use_preferred_joint_config, SPValue::UNKNOWN));
    let state = state.add(assign!(preferred_joint_config, SPValue::UNKNOWN));
    let state = state.add(assign!(use_payload, SPValue::UNKNOWN));
    let state = state.add(assign!(payload, SPValue::UNKNOWN));
    let state = state.add(assign!(baseframe_id, SPValue::UNKNOWN));
    let state = state.add(assign!(faceplate_id, SPValue::UNKNOWN));
    let state = state.add(assign!(goal_feature_id, SPValue::UNKNOWN));
    let state = state.add(assign!(tcp_id, SPValue::UNKNOWN));
    let state = state.add(assign!(root_frame_id, SPValue::UNKNOWN));
    let state = state.add(assign!(cancel_current_goal, SPValue::UNKNOWN));

    state
}
