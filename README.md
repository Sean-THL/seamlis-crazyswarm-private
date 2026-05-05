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
docker exec -it DASC-Crazy-flie-ros1-bag bash


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
