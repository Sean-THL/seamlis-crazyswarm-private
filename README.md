# 🚁 DASC Crazyflie Swarm + SEAMLiS

Run a Crazyflie swarm with SEAMLiS exploration using ROS1 inside Docker.

---

## 📦 Setup

Start the Docker container:

```bash
cd DASC_Crazyflie
docker compose up -d
docker exec -it DASC-Crazy-flie-ros1-bag bash
```

All commands below are executed inside the container.

---

## 🚀 Launch Instructions

Open multiple terminals inside the container.

---

### Terminal 1: Select Crazyflies

```bash
cd $SHORT
python3 chooser.py
```

Use the GUI to select the Crazyflies that will be used in the experiment.

After selecting the Crazyflies:

1. Click **Reboot**.
2. Wait for the Crazyflies to reboot.
3. Press `Ctrl + C` to exit the GUI.

---

### Terminal 2: Start Crazyswarm

```bash
roslaunch crazyswarm hover_and_swarm.launch
```

This starts the Crazyswarm launch file for hovering and swarm operation.

---

### Terminal 3: Publish Crazyflie State

```bash
cd $SHORT
python3 cf_state_publisher.py
```

This publishes the Crazyflie state information for the exploration controller.

---

### Terminal 4: Run SEAMLiS Exploration

```bash
cd seamlis
python3 exploration_swarm.py
```

This runs the SEAMLiS exploration controller for the Crazyflie swarm.

---

### Terminal 5: Record ROS Bag (Optional)

To record all ROS topics:

```bash
rosbag record -a
```

To record selected topics only:

```bash
rosbag record /cf/state /cf_reference /tf
```

---

## 🔁 System Pipeline

```text
Vicon → cf_state_publisher.py → SEAMLiS → Crazyflie reference → Crazyflie
```

The Vicon system provides the Crazyflie pose.  
The state publisher sends the state into ROS.  
SEAMLiS computes the exploration and control reference.  
The Crazyflie tracks the generated reference.

---

## 📁 Key Files

```text
DASC_Crazyflie/
├── chooser.py
├── cf_state_publisher.py
├── seamlis/
│   └── exploration_swarm.py
├── docker-compose.yml
└── README.md
```

---

## ⚠️ Troubleshooting

### No Vicon or TF data

Check whether VRPN/Vicon topics are available:

```bash
rostopic list | grep vrpn
```

If no topics appear, make sure the Vicon/VRPN system is running and connected.

---

### Crazyflie does not move

Check the following:

- The Crazyflie was selected in `chooser.py`
- **Reboot** was clicked after selection
- The radio connection is working
- The Crazyswarm launch file is running
- The reference topic is being published

---

### Exploration finishes too quickly

Try reducing:

- `cam_range`
- `v_max`
- `a_max`

---

### Motion is too aggressive or unstable

Try reducing:

- `v_max`
- `a_max`

You may also need to tune the CBF/MPC controller gains.

---

## 🧪 Notes

- Run each launch command in a separate terminal inside the Docker container.
- Start with low velocity and acceleration limits during real-world experiments.
- Match the exploration environment size to the physical lab space.
- Use rosbag recording for debugging and experiment analysis.
