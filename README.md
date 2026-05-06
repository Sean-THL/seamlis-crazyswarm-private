# 🚁 DASC Crazyflie Swarm + SEAMLiS

Run a Crazyflie swarm with SEAMLiS exploration using ROS1 inside Docker.

This repository integrates the DASC Crazyflie control stack with SEAMLiS, allowing Crazyflie drones to perform exploration and trajectory-following experiments in a Vicon-tracked environment.

Two operation modes are supported:

1. **Closed-loop SEAMLiS control with Vicon feedback**
2. **SEAMLiS trajectory generation + Crazyflie trajectory following**

---

## 📦 Setup

Start the Docker container:

```bash
cd DASC_Crazyflie
docker compose up -d
docker exec -it dasc-crazyflie-ros-1 bash
```

All commands below should be executed inside the Docker container.

---

## 🚀 Operation Mode 1: Closed-Loop SEAMLiS Control

In this mode, the Vicon system provides the real-time Crazyflie state.  
The state is published into ROS, SEAMLiS receives the current state, computes the control input, and sends the generated reference back to the Crazyflie.

This is the main workflow for real-time exploration experiments.

### 🔁 Mode 1 System Pipeline

```text
Vicon → cf_state_publisher.py → SEAMLiS → Crazyflie reference → Crazyflie
```

The workflow is:

1. Vicon tracks the Crazyflie position and orientation (only when attaching 3 or more markers).
2. `cf_state_publisher.py` publishes the Crazyflie state (x,y,z) to ROS.
3. SEAMLiS reads the current Crazyflie state.
4. SEAMLiS computes the exploration/control reference.
5. The Crazyflie tracks the generated reference trajectory.

---

## 🧭 Mode 1 Launch Instructions

Open multiple terminals inside the Docker container.

### Terminal 1: Select Crazyflies

```bash
cd $SHORT
python3 chooser.py
```

Use the GUI to select the Crazyflies that will be used in the experiment.
And check the .yaml file to place them in the inital position (e.g. cf6 in (-1.0 ,0.0, pi/2)), yaw angle can be ignored if drone has single marker

After selecting the Crazyflies:

1. Click **Reboot**.
2. Wait for the Crazyflies to reboot.
3. If /timeout shows in terminal, check the DASC-Crazyflie documentation
4. Press `Ctrl + C` to exit the GUI.

---

### Terminal 2: Start Crazyswarm

```bash
roslaunch crazyswarm hover_swarm.launch
```

This starts the Crazyswarm launch file for hovering and swarm operation.

At this stage, the Vicon system should start tracking the markers on the Crazyflies.
Can check whether Vicon gives the information Crazyflies by:
```bash
rostopic echo -n 3 /cf{id}/state
```

---

### Terminal 3: Publish Crazyflie State

```bash
cd $SHORT
python3 cf_state_publisher.py
```

This publishes the Crazyflie state information to ROS so that SEAMLiS can receive the current robot state.

The published state is used by SEAMLiS as feedback for closed-loop exploration and control.

---

### Terminal 4: Run SEAMLiS Exploration

```bash
cd seamlis
python3 exploration_swarm.py
```
--attitude for attitude controller selection
--num_agent for number of cf operating
--w_max for max yaw selection

This runs the SEAMLiS exploration controller for the Crazyflie swarm.

SEAMLiS computes the desired control reference based on:

- Current Crazyflie state
- Exploration map
- Environment information
- Obstacle information
- Controller settings
- Safety constraints

---

### Terminal 5: Record ROS Bag Optional

To record all ROS topics:

```bash

  mkdir -p /root/crazyswarm/bags
  cd /root/crazyswarm/bags/

  rosbag record \
    -O seamlis_simple_$(date +%Y%m%d_%H%M%S).bag \
    /cf6/state \
    /cf12/state \
    /cf6/cmd_position \
    /cf12/cmd_position \
    /tf \
    /tf_static \
    /rosout

```

To record selected topics only:

```bash
rosbag record /cf/state /cf_reference /tf
```

ROS bag recording is useful for debugging, experiment analysis, and replaying data after the flight.

---

## 🚀 Operation Mode 2: SEAMLiS Trajectory Generation + Crazyflie Tracking

In this mode, SEAMLiS is used to generate the drone trajectory first.  
The Crazyflie then follows the generated trajectory.

This method separates trajectory generation from real-time control.

### 🔁 Mode 2 System Pipeline

```text
SEAMLiS → Generated trajectory → Crazyflie trajectory tracking → Crazyflie
```

The workflow is:

1. SEAMLiS generates a desired path or trajectory.
2. The trajectory is converted into Crazyflie-compatible references.
3. The Crazyflie receives the trajectory commands.
4. The Crazyflie follows the planned trajectory.

---

## 🧠 Mode 2 Concept

Instead of using SEAMLiS as a fully closed-loop online controller, SEAMLiS can be used as a trajectory planner.

The general procedure is:

1. Define the exploration environment in SEAMLiS.
2. Generate the desired trajectory for each Crazyflie.
3. Save or publish the generated trajectory.
4. Convert the trajectory into Crazyflie reference commands.
5. Send the trajectory to the Crazyflie.
6. Let the Crazyflie follow the planned path.

---

## 🧭 Example Mode 2 Workflow

```text
SEAMLiS generates waypoints
        ↓
Waypoints are converted into Crazyflie references
        ↓
Crazyflie receives trajectory commands
        ↓
Crazyflie follows the planned path
```

---

## ✅ When to Use Mode 2

Use Mode 2 when:

- You want to test a pre-planned trajectory.
- You do not need online replanning.
- You want the Crazyflie to follow a fixed SEAMLiS-generated path.
- You want to separate trajectory generation from real-time feedback control.
- You want a simpler experiment setup before running full closed-loop exploration.
- You want to validate the trajectory before flying in a real-world experiment.

---

## 📁 Repository Structure

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

## 📡 ROS Topics

The following topics are commonly used during the experiment.

```text
/cf/state
/cf_reference
/tf
/vrpn_client_node/<crazyflie_name>/pose
```

Topic names may vary depending on the Crazyflie configuration and Vicon rigid body naming.

### Check Available Topics

```bash
rostopic list
```

### Check Crazyflie State

```bash
rostopic echo /cf/state
```

### Check Crazyflie Reference

```bash
rostopic echo /cf_reference
```

### Check TF

```bash
rostopic echo /tf
```

---

Process with ROS bag
  1. Record bag

  Make sure cf_state_publisher_node.py is running if you want real measured trajectories.

  rosbag record \
    -O seamlis_test.bag \
    /cf6/state \
    /cf12/state \
    /cf6/cmd_position \
    /cf12/cmd_position \
    /tf \
    /tf_static \
    /rosout

  2. Check bag

  rosbag info seamlis_test.bag

  Confirm it contains:

  /cf6/state
  /cf12/state
  /cf6/cmd_position
  /cf12/cmd_position

  3. Extract CSVs

  rostopic echo -b seamlis_test.bag -p /cf6/state > cf6_state.csv
  rostopic echo -b seamlis_test.bag -p /cf6/cmd_position > cf6_cmd_position.csv
  rostopic echo -b seamlis_test.bag -p /cf12/state > cf12_state.csv
  rostopic echo -b seamlis_test.bag -p /cf12/cmd_position > cf12_cmd_position.csv

  4. Put CSVs in one folder

  Example:

  /home/robin/Safe_exploratin/DASC-Crazyflie/crazyswarm/bags/trajectory_replay/gatekeeper

  Required names:

  cf6_state.csv
  cf6_cmd_position.csv
  cf12_state.csv
  cf12_cmd_position.csv

  5. Visualize animation

  cd /home/robin/Safe_exploratin/seamlis

  uv run python visualize_rosbag_csv.py \
    --bag_dir /home/robin/Safe_exploratin/DASC-Crazyflie/crazyswarm/bags/trajectory_replay/
  gatekeeper \
    --cf_ids 6,12 \
    --animate \
    --show_cmd

  6. Save video

  uv run python visualize_rosbag_csv.py \
    --bag_dir /home/robin/Safe_exploratin/DASC-Crazyflie/crazyswarm/bags/trajectory_replay/
  gatekeeper \
    --cf_ids 6,12 \
    --animate \
    --show_cmd \
    --output trajectory_gatekeeper.mp4 \
    --fps 20 \
    --no_show


## ⚠️ Troubleshooting

### No Vicon or TF Data

Check whether VRPN/Vicon topics are available:

```bash
rostopic list | grep vrpn
```

You can also check TF topics:

```bash
rostopic list | grep tf
```

If no topics appear, make sure:

- The Vicon system is running.
- The VRPN client is connected.
- The Crazyflie markers are visible to the Vicon cameras.
- The correct Crazyflie rigid body names are being used.
- The Vicon rigid body name matches the name expected by the ROS/Crazyswarm configuration.

---

### Crazyflie Does Not Move

Check the following:

- The Crazyflie was selected in `chooser.py`.
- **Reboot** was clicked after selection.
- The Crazyflie battery is charged.
- The radio connection is working.
- The Crazyswarm launch file is running.
- The reference topic is being published.
- Vicon is tracking the Crazyflie correctly.
- SEAMLiS is running without errors.
- The generated reference is within the physical lab boundary.

Check all published topics:

```bash
rostopic list
```

Check whether references are being published:

```bash
rostopic echo /cf_reference
```

---

### Exploration Finishes Too Quickly

If exploration finishes too quickly, try reducing:

- `cam_range`
- `v_max`
- `a_max`

You may also need to increase the exploration environment size so that it better matches the physical lab space.

---

### Motion Is Too Aggressive or Unstable

If the Crazyflie motion is too aggressive, unstable, or unsafe, try reducing:

- `v_max`
- `a_max`

You may also need to tune:

- CBF controller gains
- MPC horizon
- MPC cost weights
- Safety distance
- Reference update rate
- Goal reached threshold

Start with conservative velocity and acceleration limits during real-world experiments.

---

### Vicon Frame or World Frame Error

If you see an error related to missing frames, such as:

```text
world passed to lookupTransform argument target_frame does not exist
```

Check whether the required TF frames are being published:

```bash
rostopic echo /tf
```

You can also generate a TF frame diagram:

```bash
rosrun tf view_frames
```

Make sure the following frames are consistent:

- Vicon world frame
- Crazyswarm world frame
- Crazyflie body frame
- SEAMLiS map frame

---

### Reference Topic Is Not Published

If `/cf_reference` is not being published, check:

- SEAMLiS is running.
- The Crazyflie state is being received.
- The controller is not stopped by safety constraints.
- The robot has not already reached the goal.
- The topic name in the publisher matches the topic expected by Crazyswarm.

Use:

```bash
rostopic list
rostopic echo /cf/state
rostopic echo /cf_reference
```

---

## 🎥 ROS Bag Recording

To record all topics:

```bash
rosbag record -a
```

To record selected topics:

```bash
rosbag record /cf/state /cf_reference /tf
```

Recommended topics for debugging:

```bash
rosbag record /cf/state /cf_reference /tf /vrpn_client_node/cf1/pose
```

Adjust the Vicon topic name based on your Crazyflie rigid body name.

---

## 🧪 Experiment Notes

- Run each launch command in a separate terminal inside the Docker container.
- Start with low velocity and acceleration limits during real-world experiments.
- Match the SEAMLiS exploration environment size to the physical lab space.
- Make sure Vicon is tracking the Crazyflie before running closed-loop experiments.
- Use rosbag recording for debugging and experiment analysis.
- Verify topic names before running the full experiment.
- Test with one Crazyflie first before running the full swarm.
- Increase controller aggressiveness only after stable low-speed tests.

---

## ✅ Recommended Experiment Procedure

For safety, use the following order during real-world experiments:

1. Start the Docker container.
2. Select and reboot the Crazyflies using `chooser.py`.
3. Start Crazyswarm.
4. Confirm that Vicon is tracking the Crazyflies.
5. Run the Crazyflie state publisher.
6. Check that Crazyflie states are being published.
7. Run SEAMLiS.
8. Check that reference commands are being published.
9. Start with low velocity and acceleration limits.
10. Record rosbag data for later debugging.
11. Increase controller aggressiveness only after stable low-speed tests.

---

## 📌 Mode Comparison

| Mode | Description | Best For |
|---|---|---|
| Mode 1 | Vicon provides real-time Crazyflie state and SEAMLiS computes control online | Closed-loop exploration |
| Mode 2 | SEAMLiS generates the trajectory and Crazyflie follows it | Pre-planned trajectory tracking |

---

## 📌 Summary

This project connects DASC Crazyflie swarm control with SEAMLiS exploration.

Two operation methods are supported:

### Mode 1: Closed-Loop Control

```text
Vicon → Crazyflie state publisher → SEAMLiS → Crazyflie reference → Crazyflie
```

This mode is used for real-time exploration and feedback control.

### Mode 2: Trajectory Following

```text
SEAMLiS trajectory generation → Crazyflie trajectory tracking → Crazyflie
```

This mode is used for testing pre-generated trajectories and planned paths.

Mode 1 is the main workflow for real-time safe exploration.  
Mode 2 is useful for testing planned trajectories before running full closed-loop experiments.
