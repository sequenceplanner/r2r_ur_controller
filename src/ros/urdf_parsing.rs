use crate::URDFParameters;
use std::process::{Command, Stdio};


pub fn convert_xacro_to_urdf(args: URDFParameters) -> Option<String> {

    let output = Command::new("xacro")
        .arg(args.description_file)
        .arg(format!("safety_limits:={}", args.safety_limits))
        .arg(format!("safety_pos_margin:={}", args.safety_pos_margin))
        .arg(format!("safety_k_position:={}", args.safety_k_position))
        .arg(format!("name:={}", args.name))
        .arg(format!("ur_type:={}", args.ur_type))
        .arg(format!("prefix:={}", args.tf_prefix))
        .stdout(Stdio::piped())
        .output().expect("Failed to generate URDF with Xacro");

    if !output.status.success() {
        eprintln!("Xacro process failed with exit code: {:?}", output.status.code());
        eprintln!("stderr: {}", String::from_utf8_lossy(&output.stderr));
        return None;  // or handle error
    }

    // Convert stdout to a string, which should be the rendered URDF
    let robot_description = String::from_utf8_lossy(&output.stdout).into_owned();
    println!("Generated robot description, successfull.");
    // println!("{:?}", robot_description);
    Some(robot_description)

}

#[test]
fn test_xacro() {
    use std::path::PathBuf;
    // let robot_id = std::env::var("ROBOT_ID").expect("ROBOT_ID is not set");
    let urdf_dir = std::env::var("URDF_DIR").expect("URDF_DIR is not set");
    // let templates_dir = std::env::var("TEMPLATES_DIR").expect("TEMPLATES_DIR is not set");

    let mut path = PathBuf::from(&urdf_dir);
    path.push("ur.urdf.xacro");

    let urdf_path = path.to_string_lossy().to_string();

    let mut params = URDFParameters::default();

    params.description_file = urdf_path; //format!("{}/src/description/urdf/ur.urdf.xacro", manifest_dir);
    let urdf = convert_xacro_to_urdf(params);
    println!("{}", urdf.unwrap())
    // println!("urdf: {:?}", urdf.unwrap());

}