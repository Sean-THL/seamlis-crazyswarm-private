# SEAMLiS Crazyswarm Integration

Private working repository for the SEAMLiS hardware exploration pipeline with DASC Crazyflie/Crazyswarm support.

This repository vendors the current local working copies of:

- `seamlis/`
- `DASC-Crazyflie/`

Generated ROS build outputs, rosbags, CSV exports, animations, caches, and local scratch files are intentionally ignored.

⚙️ Dependencies
System Requirements
Docker + Docker Compose
ROS1 (Noetic, inside container)
Crazyflie + Crazyswarm
Vicon / VRPN system
🚀 Setup

Start the Docker environment:

cd DASC_Crazyflie
docker compose up -d
docker exec -it DASC-Crazy-flie-ros1-bag bash

All subsequent commands are executed inside the container.

🧪 Execution Pipeline

The system requires multiple concurrent processes. Each should be run in a separate terminal inside the container.

1. Crazyflie Selection
cd $SHORT
python3 chooser.py
Select active Crazyflies
Click Reboot to apply configuration
Exit using Ctrl + C
2. Swarm Initialization
roslaunch crazyswarm hover_and_swarm.launch

This initializes:

Radio communication
Low-level control interface
Swarm coordination
3. State Publishing
cd $SHORT
python3 cf_state_publisher.py

This node:

Subscribes to Vicon/VRPN pose data
Publishes processed state to ROS
4. Exploration Execution
cd seamlis
python3 exploration_swarm.py

This module:

Runs SEAMLiS exploration
Computes safe control inputs (CBF / MPC)
Generates reference trajectories
5. (Optional) Data Recording
rosbag record -a

Or selective recording:

rosbag record /cf/state /cf_reference /tf
For launching the this:

Initially, activate the docker 
cd DASC_Crazyflie
docker compose up -d
docker exec -it DASC-Crazy-flie-ros1-bag bash

The below are run inside the container:


Terminal 1:
cd $SHORT 
python3 chooser.py

Above is to choose the crazyflies operating, click reboot after selection.
Ctrl C to exit the GUI


Terminal 2: 
roslaunch crazyswarm hover_and_swarm.launch

Terminal 3:
cd $SHORT 
python3 cf_state_publisher.py

Terminal 4:
cd seamlis
python3 exploration_swarm.py

(Optional)
Terminal 5:
rosbag recording
