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

# 🚀 Operation Mode 1: Closed-Loop SEAMLiS Control

In this mode, the Vicon system provides the real-time Crazyflie state.

The state is published into ROS, SEAMLiS receives the current state, computes the control input, and sends the generated reference back to the Crazyflie.

This is the main workflow for real-time exploration experiments.

---

## 🔁 Mode 1 System Pipeline

```text
Vicon → cf_state_publisher.py → SEAMLiS → Crazyflie reference → Crazyflie
```

The workflow is:

1. Vicon tracks the Crazyflie position and orientation.
   - For full position and orientation tracking, attach **3 or more markers**.
   - If the drone only has a single marker, yaw angle can be ignored.
2. `cf_state_publisher.py` publishes the Crazyflie state to ROS.
3. SEAMLiS reads the current Crazyflie state.
4. SEAMLiS computes the exploration/control reference.
5. The Crazyflie tracks the generated reference trajectory.

---

## 🧭 Mode 1 Launch Instructions

Open multiple terminals inside the Docker container.

---

## Terminal 1: Select Crazyflies

```bash
cd $SHORT
python3 chooser.py
```

Use the GUI to select the Crazyflies that will be used in the experiment.

Before running the experiment, also check the `.yaml` file and place the Crazyflies in their initial positions.

Example:

```yaml
cf6:
  initialPosition: [-1.0, 0.0, 0.0]
```

If the Crazyflie only has a single marker, yaw angle can be ignored.

After selecting the Crazyflies:

1. Click **Reboot**.
2. Wait for the Crazyflies to reboot.
3. If `/timeout` appears in the terminal, check the DASC-Crazyflie documentation.
4. Press `Ctrl + C` to exit the GUI.

---

## Terminal 2: Start Crazyswarm

```bash
roslaunch crazyswarm hover_swarm.launch
```

This starts the Crazyswarm launch file for hovering and swarm operation.

At this stage, the Vicon system should start tracking the markers on the Crazyflies.

To check whether Vicon is providing Crazyflie state information, use:

```bash
rostopic echo -n 3 /cf{id}/state
```

Example:

```bash
rostopic echo -n 3 /cf6/state
```

---

## Terminal 3: Publish Crazyflie State

```bash
cd $SHORT
python3 cf_state_publisher.py
```

This publishes the Crazyflie state information to ROS so that SEAMLiS can receive the current robot state.

The published state is used by SEAMLiS as feedback for closed-loop exploration and control.

---

## Terminal 4: Run SEAMLiS Exploration

```bash
cd seamlis
python3 exploration_swarm.py
```

Useful command-line arguments:

```text
--attitude   Select the attitude controller
--num_agent  Set the number of Crazyflies operating
--w_max      Set the maximum yaw rate
```

Example:

```bash
python3 exploration_swarm.py --num_agent 2 --attitude gatekeeper --w_max 1.0
```

This runs the SEAMLiS exploration controller for the Crazyflie swarm.

SEAMLiS computes the desired control reference based on:

- Current Crazyflie state
- Exploration map
- Environment information
- Obstacle information
- Controller settings
- Safety constraints

---

## Terminal 5: Record ROS Bag Optional

Create a directory for ROS bag recording:

```bash
mkdir -p /root/crazyswarm/bags
cd /root/crazyswarm/bags/
```

Record selected topics:

```bash
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

To record all ROS topics:

```bash
rosbag record -a
```

ROS bag recording is useful for debugging, experiment analysis, and replaying data after the flight.

---

# 🚀 Operation Mode 2: SEAMLiS Trajectory Generation + Crazyflie Tracking

In this mode, SEAMLiS is used to generate a trajectory first.  
The Crazyflies then follow the pre-generated trajectory using Crazyswarm.

This mode separates trajectory generation from real-time SEAMLiS feedback control.

---

## 🔁 Mode 2 System Pipeline

```text
SEAMLiS trajectory CSV → follow_sim_trajectory_node.py → Crazyflie cmd_position → Crazyflie
```

The workflow is:

1. Generate a trajectory from SEAMLiS.
2. Save the trajectory as a `.csv` file.
3. Start Crazyswarm.
4. Record ROS bag data.
5. Run a dry replay first.
6. If the dry run is correct, run the real trajectory replay.
7. The Crazyflies follow the generated trajectory.

---

## 🧭 Mode 2 Launch Instructions

Run this ouside of Docker to collect the trajectory:
```bash
uv run python examples/flylab.py  --attitude gatekeeper   --w_max 1.6   --output_trajectory trajectories/cf6_cf12_gatekeeper.csv  --save_anim
```
After collecting the trajectory from seamlis, open multiple terminals.
---

## Terminal 1: Start Crazyswarm

If Crazyswarm is not already running, start it first:

```bash
cd /root/crazyswarm
source /opt/ros/noetic/setup.bash
source /root/crazyswarm/ros_ws/devel/setup.bash

roslaunch crazyswarm hover_swarm.launch use_rviz:=false
```

This starts the Crazyswarm hover and swarm operation launch file.

The argument:

```bash
use_rviz:=false
```

disables RViz during trajectory replay.

---

## Terminal 2: Record ROS Bag

Open a new terminal and enter the Docker container:

```bash
docker exec -it dasc-crazyflie-ros-1 bash
```

Source the ROS environment:

```bash
source /opt/ros/noetic/setup.bash
source /root/crazyswarm/ros_ws/devel/setup.bash
```

Create a folder for the simple trajectory replay bag:

```bash
mkdir -p /root/crazyswarm/bags/trajectory_replay/simple
cd /root/crazyswarm/bags/trajectory_replay/simple
```

Record the required ROS topics:

```bash
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

This records both the measured Crazyflie states and the commanded trajectory references.

---

## Terminal 3: Dry Run Trajectory Replay

Open another terminal and enter the Docker container:

```bash
docker exec -it dasc-crazyflie-ros-1 bash
```

Go to the Crazyswarm scripts directory:

```bash
cd /root/crazyswarm/ros_ws/src/crazyswarm/scripts
```

Source the ROS environment:

```bash
source /opt/ros/noetic/setup.bash
source /root/crazyswarm/ros_ws/devel/setup.bash
```

Run the trajectory replay in dry-run mode first:

```bash
python3 follow_sim_trajectory_node.py \
  --trajectory /root/seamlis/trajectories/cf6_cf12_simple.csv \
  --cf_ids 6,12 \
  --z 0.6 \
  --rate_hz 30 \
  --time_scale 3.0 \
  --dry_run
```

The `--dry_run` flag checks the trajectory replay logic without sending real movement commands to the Crazyflies.

Use this step to verify:

- The trajectory file exists.
- The Crazyflie IDs are correct.
- The trajectory format is valid.
- The replay node can read the CSV file.
- The timing and command generation look reasonable.

---

## Terminal 3: Real Trajectory Replay

If the dry run is correct, run the real replay:

```bash
python3 follow_sim_trajectory_node.py \
  --trajectory /root/seamlis/trajectories/cf6_cf12_simple.csv \
  --cf_ids 6,12 \
  --z 0.6 \
  --rate_hz 30 \
  --time_scale 3.0
```

This sends the generated trajectory references to Crazyflies `cf6` and `cf12`.

---

## 🧾 Mode 2 Command Arguments

| Argument | Description |
|---|---|
| `--trajectory` | Path to the SEAMLiS-generated trajectory CSV file |
| `--cf_ids` | Crazyflie IDs used in the replay |
| `--z` | Fixed flight height for trajectory replay |
| `--rate_hz` | Command publishing rate |
| `--time_scale` | Time scaling factor for slowing down or speeding up the trajectory |
| `--dry_run` | Test mode that does not send real motion commands |

---

## ✅ Recommended Mode 2 Procedure

For safety, use the following order:

1. Start Crazyswarm.
2. Make sure the Crazyflies are selected and rebooted.
3. Confirm that Vicon is tracking the Crazyflies.
4. Start ROS bag recording.
5. Run the trajectory replay with `--dry_run`.
6. Check that the trajectory file and Crazyflie IDs are correct.
7. Run the real trajectory replay.
8. Stop the ROS bag recording after the experiment.
9. Analyze the recorded bag file.

---

## ⚠️ Mode 2 Safety Notes

- Always run `--dry_run` before real replay.
- Start with a safe height such as `--z 0.6`.
- Use a larger `--time_scale` value to slow down the trajectory.
- Make sure the generated trajectory stays inside the Vicon tracking space.
- Make sure the trajectory does not collide with obstacles or other Crazyflies.
- Confirm that `/cf6/state`, `/cf12/state`, `/cf6/cmd_position`, and `/cf12/cmd_position` are being recorded.

# 📁 Repository Structure

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

# 📡 ROS Topics

The following topics are commonly used during the experiment.

```text
/cf{id}/state
/cf{id}/cmd_position
/tf
/tf_static
/rosout
/vrpn_client_node/<crazyflie_name>/pose
```

Example topics:

```text
/cf6/state
/cf12/state
/cf6/cmd_position
/cf12/cmd_position
```

Topic names may vary depending on the Crazyflie configuration and Vicon rigid body naming.

---

## Check Available Topics

```bash
rostopic list
```

---

## Check Crazyflie State

```bash
rostopic echo /cf6/state
```

---

## Check Crazyflie Command Position

```bash
rostopic echo /cf6/cmd_position
```

---

## Check TF

```bash
rostopic echo /tf
```

---

# 🎥 ROS Bag Processing Workflow

This section explains how to record, check, extract, and visualize ROS bag data.

---

## Step 1: Record ROS Bag

Make sure `cf_state_publisher.py` is running if you want real measured trajectories.

```bash
rosbag record \
  -O seamlis_test.bag \
  /cf6/state \
  /cf12/state \
  /cf6/cmd_position \
  /cf12/cmd_position \
  /tf \
  /tf_static \
  /rosout
```

---

## Step 2: Check ROS Bag

```bash
rosbag info seamlis_test.bag
```

Confirm that the bag contains:

```text
/cf6/state
/cf12/state
/cf6/cmd_position
/cf12/cmd_position
```

---

## Step 3: Extract CSV Files

```bash
rostopic echo -b seamlis_test.bag -p /cf6/state > cf6_state.csv
rostopic echo -b seamlis_test.bag -p /cf6/cmd_position > cf6_cmd_position.csv

rostopic echo -b seamlis_test.bag -p /cf12/state > cf12_state.csv
rostopic echo -b seamlis_test.bag -p /cf12/cmd_position > cf12_cmd_position.csv
```

---

## Step 4: Put CSV Files in One Folder

Example folder:

```text
/home/robin/Safe_exploratin/DASC-Crazyflie/crazyswarm/bags/trajectory_replay/gatekeeper
```

Required file names:

```text
cf6_state.csv
cf6_cmd_position.csv
cf12_state.csv
cf12_cmd_position.csv
```

---

## Step 5: Visualize Animation

Go to the SEAMLiS directory:

```bash
cd /home/robin/Safe_exploratin/seamlis
```

Run the visualization script:

```bash
uv run python visualize_rosbag_csv.py \
  --bag_dir /home/robin/Safe_exploratin/DASC-Crazyflie/crazyswarm/bags/trajectory_replay/gatekeeper \
  --cf_ids 6,12 \
  --animate \
  --show_cmd
```

---

## Step 6: Save Video

```bash
uv run python visualize_rosbag_csv.py \
  --bag_dir /home/robin/Safe_exploratin/DASC-Crazyflie/crazyswarm/bags/trajectory_replay/gatekeeper \
  --cf_ids 6,12 \
  --animate \
  --show_cmd \
  --output trajectory_gatekeeper.mp4 \
  --fps 20 \
  --no_show
```

---

# ⚠️ Troubleshooting

---

## No Vicon or TF Data

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

## Crazyflie Does Not Move

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
rostopic echo /cf6/cmd_position
```

---

## Exploration Finishes Too Quickly

If exploration finishes too quickly, try reducing:

- `cam_range`
- `v_max`
- `a_max`

You may also need to increase the exploration environment size so that it better matches the physical lab space.

---

## Motion Is Too Aggressive or Unstable

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

## Vicon Frame or World Frame Error

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

## Reference Topic Is Not Published

If the Crazyflie reference topic is not being published, check:

- SEAMLiS is running.
- The Crazyflie state is being received.
- The controller is not stopped by safety constraints.
- The robot has not already reached the goal.
- The topic name in the publisher matches the topic expected by Crazyswarm.

Use:

```bash
rostopic list
rostopic echo /cf6/state
rostopic echo /cf6/cmd_position
```

---

# 🧪 Experiment Notes

- Run each launch command in a separate terminal inside the Docker container.
- Start with low velocity and acceleration limits during real-world experiments.
- Match the SEAMLiS exploration environment size to the physical lab space.
- Make sure Vicon is tracking the Crazyflie before running closed-loop experiments.
- Use ROS bag recording for debugging and experiment analysis.
- Verify topic names before running the full experiment.
- Test with one Crazyflie first before running the full swarm.
- Increase controller aggressiveness only after stable low-speed tests.

---

# ✅ Recommended Experiment Procedure

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
10. Record ROS bag data for later debugging.
11. Increase controller aggressiveness only after stable low-speed tests.

---

# 📌 Mode Comparison

| Mode | Description | Best For |
|---|---|---|
| Mode 1 | Vicon provides real-time Crazyflie state and SEAMLiS computes control online | Closed-loop exploration |
| Mode 2 | SEAMLiS generates the trajectory and Crazyflie follows it | Pre-planned trajectory tracking |

---

# 📌 Summary

This project connects DASC Crazyflie swarm control with SEAMLiS exploration.

Two operation methods are supported:

---

## Mode 1: Closed-Loop Control

```text
Vicon → Crazyflie state publisher → SEAMLiS → Crazyflie reference → Crazyflie
```

This mode is used for real-time exploration and feedback control.

---

## Mode 2: Trajectory Following

```text
SEAMLiS trajectory generation → Crazyflie trajectory tracking → Crazyflie
```

This mode is used for testing pre-generated trajectories and planned paths.

Mode 1 is the main workflow for real-time safe exploration.

Mode 2 is useful for testing planned trajectories before running full closed-loop experiments.
