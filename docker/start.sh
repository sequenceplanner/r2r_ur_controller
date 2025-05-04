#!/bin/bash
set -e

echo "Access kasmVNC at http://<host>:6902/"

# --- Add this line to start your Rust application ---
echo "Starting Robot stuff"
# Replace 'phoxi_ci_binary_name' with the actual name of your compiled executable
/usr/local/src/r2r_ur_controller/target/release/r2r_ur_controller &
# --- End Rust application start ---

# Keep container running
tail -f /dev/null