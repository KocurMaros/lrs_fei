#!/bin/bash

# Path to the Gazebo world file
WORLD_PATH="src/LRS-FEI/worlds/fei_lrs_gazebo.world"

# Coordinates for sim_vehicle.py
LATITUDE=48.15084570555732
LONGITUDE=17.072729745416016
ALTITUDE=150
HEADING=0

# Run Gazebo with the specified world
echo "Starting Gazebo with world: $WORLD_PATH"
gazebo $WORLD_PATH &
GAZEBO_PID=$!

# Wait for Gazebo to start
sleep 5

# Run the simulation vehicle
echo "Starting simulation vehicle"
cd ../ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris --console -l $LATITUDE,$LONGITUDE,$ALTITUDE,$HEADING &
SIM_VEHICLE_PID=$!

# Wait for the simulation vehicle to start
sleep 5

# Check if sim_vehicle.py is still running
if ! kill -0 $SIM_VEHICLE_PID > /dev/null 2>&1; then
    echo "sim_vehicle.py exited unexpectedly. Exiting script."
    kill $GAZEBO_PID
    exit 1
fi

# Run the MAVROS node
echo "Starting MAVROS node"
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555 &
MAVROS_PID=$!

# Function to handle script termination
cleanup() {
    echo "Terminating processes..."
    kill $GAZEBO_PID
    kill $SIM_VEHICLE_PID
    kill $MAVROS_PID
}

# Trap script termination and call cleanup
trap cleanup EXIT

# Wait for all background processes to finish
wait $GAZEBO_PID $SIM_VEHICLE_PID $MAVROS_PID