# SEAMLiS + Crazyswarm

**Visibility-aware safety for perception-limited multi-robot exploration, from simulation to Crazyflie hardware.**

[Paper](https://arxiv.org/abs/2607.09959) · [HTML](https://arxiv.org/html/2607.09959v1) · [Project page](https://www.taekyung.me/seamlis) · [Upstream SEAMLiS](https://github.com/tkkim-robot/seamlis)

This repository integrates **SEAMLiS** (Safe Exploration for Autonomous Multi-Robot Systems Under Limited Sensing) with the ROS 1 version of [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm). SEAMLiS is an execution-layer safety framework: it leaves the exploration goal allocator and local planner intact, then filters attitude and position commands so robots can explore with finite sensing range and a limited field of view (FoV) without discovering hidden obstacles too late to avoid them.

It includes decentralized frontier and CoScan-inspired exploration, A* local planning, a gatekeeper attitude filter, MPC-CBF/CBF-QP position control, randomized simulation and benchmarking, and a Dockerized ROS Noetic/Crazyswarm hardware stack.

> [!IMPORTANT]
> The paper reports a ROS 2 hardware implementation. This repository provides a **ROS 1 Noetic/Crazyswarm implementation of the same experimental architecture**. In the hardware experiment, limited-FoV detections are generated geometrically from Vicon pose; an onboard RGB-D camera is not required.

## Why SEAMLiS?

Exploration stacks usually plan toward informative frontiers while a local controller avoids obstacles already in the map. With limited FoV, those layers can work correctly in isolation and still be unsafe together. A quadrotor can translate sideways while its sensor looks toward an information-rich region, leaving the direction of travel unobserved. If a hidden obstacle appears only after the vehicle is too close to brake, a CBF controller cannot recover feasibility retroactively.

SEAMLiS addresses this gap below the planner:

```text
local/received frontiers
          │
          ▼
 goal allocation ──► A* waypoint path ──► nominal position + yaw commands
                                                │
                    ┌───────────────────────────┴──────────────────────────┐
                    ▼                                                      ▼
      gatekeeper attitude filter                              MPC-CBF position filter
  nominal: visibility/information gain                 known + detected + robot obstacles
  backup: align sensor with velocity                                  │
                    └───────────────────────────┬──────────────────────┘
                                                ▼
                                      safe command to robot
                                                │
                          localization + finite-FoV sensing feedback
```

The safety layer only requires a waypoint path and each robot's local map, so the upstream allocator is interchangeable.

## Theory in brief

### Dynamics and sensing

Each planar robot is modeled as a double integrator with independently controlled yaw:

$$
\dot p_i=v_i,\qquad \dot v_i=a_i,\qquad \dot\theta_i=\omega_i,
$$

with bounded speed, acceleration, and yaw rate. The worst-case braking distance is

$$
d_{\mathrm{br}}=\frac{v_{\max}^2}{2a_{\max}}.
$$

The sensor footprint is a range-limited horizontal FoV sector with line-of-sight checking. Each robot accumulates its own known-free set $B_i$, known-obstacle set $O_i$, and unknown set $U_i$. Maps and obstacle lists are not assumed to be shared.

### Visibility-safety condition

Let the **critical point** $p_i^c$ be the first place where the current planned path intersects the boundary of the robot's known-free region. Let $d_i^c$ be path distance to that point, and let $\Delta\theta_i^c$ be the angular rotation required to bring it into the FoV. SEAMLiS uses

$$
h_i^{\mathrm{vis}}
=d_i^c-d_{\mathrm{br}}-\frac{v_{\max}}{\omega_{\max}}\Delta\theta_i^c.
$$

The condition $h_i^{\mathrm{vis}}\ge 0$ means the robot can rotate the critical boundary into view and still retain a full worst-case braking distance. This is the link between perception and control: motion-relevant unknown space must become visible while avoidance is still feasible.

### Gatekeeper attitude filter

The nominal yaw policy points the FoV toward the heading with the largest estimated newly observed area. This improves mapping speed but can turn the sensor away from motion. The backup policy aligns yaw with velocity.

At each update, gatekeeper predicts a candidate yaw trajectory using the nominal policy for switching duration $T_S$, followed by the backup policy for horizon $T_B$. It commits the longest candidate that keeps $h_i^{\mathrm{vis}}\ge0$ and reaches a terminal state where the critical point is visible (or absent from the look-ahead path). If no positive nominal duration is certified, the backup is applied immediately. Information-greedy sensing is therefore retained whenever it is recoverably safe.

### Positional CBF filter

For a circular obstacle $j$, the distance safety function is

$$
h_{ij}^{\mathrm{obs}}=\|p_i-o_j\|^2-r_{\mathrm{safe},ij}^2.
$$

Because acceleration appears after two derivatives, the controller applies a relative-degree-two high-order CBF condition,

$$
\ddot h+\alpha_1\dot h+\alpha_0h\ge0,
$$

inside a QP or MPC-CBF problem that stays close to nominal tracking. Pairwise constraints maintain inter-robot separation; this implementation also accounts for neighboring velocity as a moving obstacle.

Under the paper's assumptions—bounded estimation/mapping error, a visibility-safe initial condition, correct local detection, recursive gatekeeper feasibility, and a nonempty CBF-admissible input set—the visibility filter guarantees timely detection and the position filter guarantees collision avoidance. These are sufficient conditions, not a claim that arbitrary tuning or inaccurate sensing is safe.

## Paper results

The paper evaluates 1–3 robots with frontier and decentralized CoScan-inspired assignment. In 100 randomized trials per configuration, SEAMLiS completed every reported setting with zero collisions. Always tracking velocity with yaw was also collision-free but timed out in every reported setting; unconstrained visibility-promoting and constant-yaw policies were faster in some cases but collided under multi-robot limited sensing. The same qualitative behavior was demonstrated in Isaac Sim and with two physical Crazyflies.

Read the [paper](https://arxiv.org/abs/2607.09959) for the complete settings, ablations, assumptions, and proofs.

## Repository layout

```text
.
├── seamlis/
│   ├── exploration.py                 # exploration manager and local maps
│   ├── exploration_crazyswarm.py      # live ROS/Crazyswarm controller
│   ├── tracking_controller.py         # attitude + position safety stack
│   ├── position_controller.py         # moving-agent MPC-CBF extension
│   ├── algorithms/                    # frontier and CoScan-inspired allocators
│   ├── safe_control/                  # robot models, CBF/MPC, gatekeeper
│   ├── examples/                      # simulation, benchmark, lab geometry
│   └── record_sim_trajectory.py       # simulation-to-hardware CSV export
└── DASC-Crazyflie/
    ├── Dockerfile
    ├── docker-compose.yaml
    └── crazyswarm/ros_ws/src/crazyswarm/
        ├── launch/seamlis_crazyswarm.launch
        └── scripts/                   # ROS adapters, chooser, replay, emergency stop
```

## Simulation quick start

Simulation requires Python 3.9 or newer and [`uv`](https://docs.astral.sh/uv/).

```bash
git clone --recursive git@github.com:Sean-THL/seamlis-crazyswarm-private.git
cd seamlis-crazyswarm-private/seamlis
uv sync

# Default: two robots, indoor map, frontier allocation, unknown obstacles
uv run python examples/test_exploration.py --num_agent 2
```

Useful variants:

```bash
# One-robot sanity check without hidden obstacles
uv run python examples/test_exploration.py --num_agent 1 --no-unknown

# Decentralized CoScan-inspired assignment with SEAMLiS
uv run python examples/test_exploration.py \
  --num_agent 2 --algo coscan --attitude gatekeeper

# Compare an unfiltered visibility-promoting baseline
uv run python examples/test_exploration.py \
  --num_agent 2 --algo coscan --attitude visibility_area

# Save an animation
uv run python examples/test_exploration.py --num_agent 2 --save_anim
```

Primary options include `--num_agent {1,2,3}`, `--algo {frontier,coscan}`, `--attitude {gatekeeper,velocity_tracking_yaw,visibility_area,simple,...}`, `--layout {indoor,open}`, `--unknown_profile {default,stress}`, and `--use_astar`/`--no-astar`. See [`seamlis/README.md`](seamlis/README.md) for the full option list.

### Randomized benchmark

```bash
cd seamlis
uv run python examples/benchmark_random_exploration.py \
  --seed 42 --num_trials 100 --dt 0.1 --tf 300 \
  --coverage_target 0.98 --workers 8 \
  --output_dir output/benchmark_seed42
```

Runs report coverage/success, collisions, visibility or unknown-region violations, and gatekeeper acceptance statistics.

## Hardware implementation

### Experimental setup

| Component | Paper configuration |
|---|---|
| Vehicles | 2 × Crazyflie quadrotors |
| Localization | Vicon motion capture |
| Workspace | approximately 3.35 m × 3.50 m |
| Flight plane | fixed height, $z=1.0$ m |
| Robot radius | 0.075 m |
| Hidden obstacle | circular, center $(0,0)$ in Vicon, radius 0.30 m |
| Modeled sensor | 70° horizontal FoV, 0.25 m range |
| Limits | $v_{\max}=1.35$ m/s, $a_{\max}=1.5$ m/s² |
| Position safety | MPC-CBF, horizon 10, $\alpha_0=\alpha_1=0.55$ |
| Attitude safety | nominal horizon 0.4 s, backup horizon 1.8 s |
| Communication | states and frontier information; no maps or obstacle lists |

The lab geometry is configured in [`seamlis/examples/flylab.py`](seamlis/examples/flylab.py). It converts between Vicon and planner frames, defines the flight boundary, and places the hidden obstacle. **Measure your workspace and update these constants before flying.** Repository defaults may differ from paper parameters: for example, the current live profile uses a 0.8 m modeled camera range.

### Data and command path

```text
Vicon
  └─► Crazyswarm TF (/world → /cf<ID>)
        └─► cf_state_publisher_node.py → /cf<ID>/state
              └─► exploration_crazyswarm.py
                    ├─ local FoV/map + frontier/A*
                    ├─ gatekeeper yaw filter
                    └─ MPC-CBF position filter
                          └─► /cf<ID>/cmd_position → radio → Crazyflie
```

The live controller estimates planar velocity from Vicon, synchronizes the SEAMLiS model to measured state, projects acceleration/yaw outputs to bounded position steps, converts back to the Vicon frame, and publishes a fixed-height command. The launch file publishes measured yaw by default; set `publish_current_yaw:=false` to command gatekeeper-predicted yaw.

### Prerequisites

- Ubuntu host with Docker Engine and Compose
- Crazyradio PA/2.1 hardware and Bitcraze udev permissions
- Crazyflie 2.x vehicles with correctly installed propellers and charged batteries
- Vicon on the same network, with a `cf<ID>` rigid body for every vehicle
- a clear, netted flight area and a physical emergency-stop procedure

> [!CAUTION]
> This is research flight software. Validate in simulation, test with propellers removed, use conservative limits, keep the flight volume clear, and have an operator ready to stop/land every vehicle. The guarantee does not cover configuration errors, stale mocap, packet loss, unmodeled dynamics, or solver failure.

### 1. Configure vehicles and Vicon

Edit:

- `DASC-Crazyflie/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml` for available IDs, channels, types, and initial positions;
- `.../launch/crazyflies.yaml` for vehicles active in this run (the chooser updates it); and
- `.../launch/hover_swarm.launch` for `motion_capture_host_name` and firmware estimator/controller.

The launch file currently points to `192.168.1.115`; change it for your Vicon server.

### 2. Build the container and Crazyswarm

```bash
cd DASC-Crazyflie
xhost +local:docker
docker compose build
docker compose up -d
docker compose exec ros bash
```

Inside the container:

```bash
cd /root/crazyswarm
./build.sh
source ros_ws/devel/setup.bash
```

Compose mounts Crazyswarm at `/root/crazyswarm`, SEAMLiS at `/root/seamlis`, exposes USB devices, and uses host networking for ROS and Vicon.

### 3. Select active Crazyflies

```bash
cd /root/crazyswarm/ros_ws/src/crazyswarm/scripts
python3 chooser.py
```

Select the vehicles, click **Reboot**, wait for reconnection, close the chooser, and verify `launch/crazyflies.yaml`.

### 4. Preflight

Start state tracking without exploration:

```bash
roslaunch crazyswarm seamlis_crazyswarm.launch \
  run_exploration:=false use_rviz:=false
```

In another container shell:

```bash
rostopic echo -n 1 /cf6/state       # replace 6 with an active ID
rosparam get /crazyflies
rosnode list
```

Confirm that physical and reported poses agree, axes match `flylab.py`, the obstacle is at its configured coordinate, and no vehicle starts inside an obstacle or outside the workspace.

### 5. Run live SEAMLiS

Stop the preflight launch, clear the workspace, and run:

> [!WARNING]
> This integrated launch begins publishing live position commands as soon as all configured state topics are ready. Start it only when the vehicles, operator, and flight volume are ready for motion.

```bash
roslaunch crazyswarm seamlis_crazyswarm.launch \
  use_rviz:=false \
  algo:=frontier \
  attitude:=gatekeeper \
  pos_controller:=mpc_cbf \
  publish_current_yaw:=false
```

The agent count is inferred from `/crazyflies`; if supplied, `num_agent` must match. To attach to an already-running server:

```bash
roslaunch crazyswarm seamlis_crazyswarm.launch \
  start_crazyswarm:=false load_cf_params:=true use_rviz:=false
```

Record a trial in a separate shell:

```bash
rosbag record -O seamlis_trial.bag \
  /tf /tf_static /seamlis/exploration_done \
  /cf6/state /cf6/cmd_position \
  /cf7/state /cf7/cmd_position
```

Replace the IDs. On reaching the coverage target, the controller publishes `true` on `/seamlis/exploration_done`. The live node sends `/cmd_position` directly and does **not** automatically land; land through your established Crazyswarm procedure after it stops.

### Staged workflow: simulate, inspect, replay

Generate a hardware-frame CSV without flying:

```bash
cd /root/seamlis
python3 record_sim_trajectory.py \
  --cf_ids 6,7 --num_agent 2 \
  --algo frontier --attitude gatekeeper \
  --output trajectories/seamlis_sim_trajectory.csv
```

Inspect with dry-run first:

```bash
rosrun crazyswarm follow_sim_trajectory_node.py \
  --trajectory /root/seamlis/trajectories/seamlis_sim_trajectory.csv \
  --cf_ids 6,7 --dry_run
```

Remove `--dry_run` only after preflight, with vehicles at the expected initial positions and an appropriate takeoff/landing procedure.

### Process and visualize ROS bags

Inspect a recorded bag before extracting data:

```bash
rosbag info seamlis_trial.bag
```

Export measured and commanded positions for each vehicle (replace IDs as needed):

```bash
mkdir -p rosbag_csv
rostopic echo -b seamlis_trial.bag -p /cf6/state > rosbag_csv/cf6_state.csv
rostopic echo -b seamlis_trial.bag -p /cf6/cmd_position > rosbag_csv/cf6_cmd_position.csv
rostopic echo -b seamlis_trial.bag -p /cf7/state > rosbag_csv/cf7_state.csv
rostopic echo -b seamlis_trial.bag -p /cf7/cmd_position > rosbag_csv/cf7_cmd_position.csv
```

From `seamlis/`, render an interactive comparison:

```bash
uv run python visualize_rosbag_csv.py \
  --bag_dir ../rosbag_csv \
  --cf_ids 6,7 \
  --animate --show_cmd --show_fov
```

Or save a headless MP4:

```bash
uv run python visualize_rosbag_csv.py \
  --bag_dir ../rosbag_csv \
  --cf_ids 6,7 \
  --animate --show_cmd --show_fov \
  --output trajectory_gatekeeper.mp4 --fps 20 --no_show
```

## Troubleshooting

| Symptom | Checks |
|---|---|
| No `/cf<ID>/state` | Vicon rigid body, server IP, `/world → /cf<ID>` TF, and IDs in `crazyflies.yaml`. |
| Crazyflie does not move | Vehicle selected/rebooted, radio recognized, server running, and `/cf<ID>/cmd_position` active. |
| Qt/`xcb` error | Use `use_rviz:=false`; for GUIs, allow local Docker X access and check `DISPLAY`. |
| Flip at takeoff | Stop; check ID/body association, propellers, battery leads, estimator convergence, and PID selection. |
| One vehicle follows another's pose | Crazyflie IDs and Vicon rigid-body names are swapped or duplicated. |
| Exploration stops immediately | Check preflight collision logs, initial coordinates, obstacle definition, frontier count, and coverage. |
| MPC infeasible/aggressive | Reduce limits, enlarge margins, confirm map scale, and inspect solver output before flying again. |
| Hidden obstacle is missed | Check FoV/range, Vicon-to-map transform, yaw convention, and `flylab.py` obstacle coordinates. |

More radio, firmware, and Crazyswarm notes are in [`DASC-Crazyflie/README.md`](DASC-Crazyflie/README.md) and the [Crazyswarm documentation](https://crazyswarm.readthedocs.io/).

## Citation

```bibtex
@article{kim2026seamlis,
  title   = {{SEAMLiS}: Visibility-Aware Safety for Perception-Limited
             Multi-Robot Exploration},
  author  = {Kim, Taekyung and Kumar, Rahul H. and Menon, Aswin D.
             and Lin, Tzu-Hsiang and Panagou, Dimitra},
  journal = {arXiv preprint arXiv:2607.09959},
  year    = {2026}
}
```

## Acknowledgments and licensing

The flight stack is based on [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm), developed by the USC ACT Lab, and includes its MIT license in `DASC-Crazyflie/crazyswarm/LICENSE`. SEAMLiS builds on the bundled `safe_control` framework. Check each bundled component's license before redistribution; no separate top-level license is currently provided for repository-specific code.
