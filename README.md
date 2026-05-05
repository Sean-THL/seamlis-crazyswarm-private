# SEAMLiS Crazyswarm Integration

Private working repository for the SEAMLiS hardware exploration pipeline with DASC Crazyflie/Crazyswarm support.

This repository vendors the current local working copies of:

- `seamlis/`
- `DASC-Crazyflie/`

Generated ROS build outputs, rosbags, CSV exports, animations, caches, and local scratch files are intentionally ignored.


# 🚁 DASC Crazyflie Swarm + SEAMLiS

Run a Crazyflie swarm with SEAMLiS exploration using ROS1 inside Docker.

---

## 📦 Setup

Start the Docker container:

```bash
cd DASC_Crazyflie
docker compose up -d
docker exec -it DASC-Crazy-flie-ros1-bag bashInitially, activate the docker 
cd DASC_Crazyflie
docker compose up -d
docker exec -it DASC-Crazy-flie-ros1-bag bash

The below are run inside the container:

1. Select Crazyflies
cd $SHORT
python3 chooser.py
Select the Crazyflies you want to use
Click Reboot after selection
Press Ctrl + C to exit

2. Start Swarm
roslaunch crazyswarm hover_and_swarm.launch
3. Publish State (Vicon → ROS)
cd $SHORT
python3 cf_state_publisher.py
4. Run Exploration
cd seamlis
python3 exploration_swarm.py
5. (Optional) Record Data

Record all topics:

rosbag record -a

Or selected topics:

rosbag record /cf/state /cf_reference /tf
🔁 Pipeline
Vicon → cf_state_publisher → SEAMLiS → reference → Crazyflie

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
