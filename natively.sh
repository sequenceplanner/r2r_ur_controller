#!/bin/bash

# Environment configuration
export ROBOT_ID="r1"
export URDF_DIR="/home/endre/r2r_ws/src/ur_description/urdf"
export TEMPLATES_DIR="/home/endre/r2r_ws/src/r2r_ur_controller/templates"

# UR_ADDRESS: Choose one by uncommenting the desired line and commenting out the others.
# export UR_ADDRESS="192.168.1.31" # Real UR robot addr
# export UR_ADDRESS="127.0.0.1" # DockURSim natively
export UR_ADDRESS="172.17.0.1" # DockURSim Via Docker (Currently active)

# OVERRIDE_HOST: The last assignment takes precedence.
# export OVERRIDE_HOST="FALSE"
export OVERRIDE_HOST="TRUE" # This will be the effective value
export OVERRIDE_HOST_ADDRESS="172.17.0.1"

echo "Environment variables set:"
echo "ROBOT_ID=${ROBOT_ID}"
echo "URDF_DIR=${URDF_DIR}"
echo "TEMPLATES_DIR=${TEMPLATES_DIR}"
echo "UR_ADDRESS=${UR_ADDRESS}"
echo "OVERRIDE_HOST=${OVERRIDE_HOST}"
echo "OVERRIDE_HOST_ADDRESS=${OVERRIDE_HOST_ADDRESS}"