use serde::{Deserialize, Serialize};
use tokio::sync::{mpsc, oneshot};

#[derive(Serialize, Deserialize)]
struct Interpretation {
    pub command: String,
    pub acceleration: f64,
    pub velocity: f64,
    pub use_execution_time: bool,
    pub execution_time: f32,
    pub use_blend_radius: bool,
    pub blend_radius: f32,
    pub use_joint_positions: bool,
    pub joint_positions: String,
    pub use_preferred_joint_config: bool,
    pub preferred_joint_config: String,
    pub use_payload: bool,
    pub payload: String,
    pub target_in_base: String,
    pub tcp_in_faceplate: String,
}