import argparse
import csv
import math
import os
import sys

import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


# FLYLAB_X_MIN = -4.0
# FLYLAB_X_MAX = 1.0
# FLYLAB_Y_MIN = -2.3
# FLYLAB_Y_MAX = 1.8

DASC_X_MIN = -2
DASC_X_MAX = 1.35
DASC_Y_MIN = -1.35
DASC_Y_MAX = 2.15

# UNKNOWN_OBS_VICON_X = 0.4
# UNKNOWN_OBS_VICON_Y = 0.0
# UNKNOWN_OBS_RADIUS = 0.305

UNKNOWN_OBS_VICON_X = 0
UNKNOWN_OBS_VICON_Y = 0
UNKNOWN_OBS_RADIUS = 0.3
# UNKNOWN_OBS_VICON_X = 0.98
# UNKNOWN_OBS_VICON_Y = 1.12
# UNKNOWN_OBS_RADIUS = 0.305


def vicon_to_map_xy(x, y):
    return float(x) - DASC_X_MIN, float(y) - DASC_Y_MIN


def map_to_vicon_xy(x, y):
    return float(x) + DASC_X_MIN, float(y) + DASC_Y_MIN


def build_indoor_exploration_env():
    env_width = DASC_X_MAX - DASC_X_MIN
    env_height = DASC_Y_MAX - DASC_Y_MIN
    e_wall = 2.0

    def _split_with_doors(start, end, door_intervals, max_seg_len=1.8):
        intervals = [(float(start), float(end))]
        for ds, de in sorted(door_intervals):
            next_intervals = []
            for s0, e0 in intervals:
                if de <= s0 or ds >= e0:
                    next_intervals.append((s0, e0))
                    continue
                if ds > s0:
                    next_intervals.append((s0, min(ds, e0)))
                if de < e0:
                    next_intervals.append((max(de, s0), e0))
            intervals = next_intervals

        split_intervals = []
        for s0, e0 in intervals:
            length = max(e0 - s0, 0.0)
            if length < 0.25:
                continue
            num_seg = max(int(np.ceil(length / max_seg_len)), 1)
            seg_len = length / num_seg
            for i in range(num_seg):
                ss = s0 + i * seg_len
                ee = s0 + (i + 1) * seg_len
                if ee - ss > 0.2:
                    split_intervals.append((ss, ee))
        return split_intervals

    def _vertical_wall(x, y_min, y_max, door_intervals, half_thickness=0.35):
        segs = _split_with_doors(y_min, y_max, door_intervals)
        return [[x, 0.5 * (s + e), half_thickness, 0.5 * (e - s), e_wall, 0.0, 1.0] for s, e in segs]

    def _horizontal_wall(y, x_min, x_max, door_intervals, half_thickness=0.35):
        segs = _split_with_doors(x_min, x_max, door_intervals)
        return [[0.5 * (s + e), y, 0.5 * (e - s), half_thickness, e_wall, 0.0, 1.0] for s, e in segs]

    # Old known-obstacle map kept for reference. It used the large positive-coordinate
    # simulation map, so it is disabled for the current VICON Flylab bounds.
    # interior_walls = []
    # interior_walls += _vertical_wall(16.0, 0.0, 18.0, door_intervals=[(1.2, 13.4), (14.8, 17.2)])
    # interior_walls += _horizontal_wall(13.0, 0.0, 7.0, door_intervals=[(1.2, 5.8)])
    # interior_walls = np.array(interior_walls, dtype=np.float64)
    #
    # known_circles = np.array(
    #     [
    #         [1.676274334947608, 14.685978826913212, 0.34888886514506134],
    #         [10.154313911104943, 6.197057802522608, 0.31567569237250975],
    #         [11.172968489948397, 13.709415691971323, 0.44],
    #         [12.840940316109451, 5.668353014220707, 0.28],
    #         [17.84911288084617, 6.673288202225253, 0.35923213699101575],
    #         [20.74134371070081, 14.657228646665182, 0.4132990115393822],
    #         [19.49278534976608, 9.762322881834, 0.3082173123587094],
    #     ],
    #     dtype=np.float64,
    # )
    # known_circles = np.hstack((known_circles, np.zeros((known_circles.shape[0], 4))))
    # known_obs = np.vstack((known_circles, interior_walls))

    known_obs = np.empty((0, 7), dtype=np.float64)

    # Define the real-flight obstacle in Vicon coordinates, then translate it
    # into the planner/simulator map frame.
    obs_x, obs_y = vicon_to_map_xy(UNKNOWN_OBS_VICON_X, UNKNOWN_OBS_VICON_Y)
    unknown_obs = np.array([[obs_x, obs_y, UNKNOWN_OBS_RADIUS]], dtype=np.float64)

    return env_width, env_height, known_obs, unknown_obs


def build_open_exploration_env():
    env_width = 24.0
    env_height = 18.0
    e_wall = 2.0

    # Open map: only a few short wall pieces, but many known/unknown obstacles.
    short_walls = np.array(
        [
            [21.0, 13.0, 1.6, 0.35, e_wall, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    known_circles = np.array(
        [
            [3.2, 3.0, 0.45],
            [3.2, 9.0, 0.45],
            [3.2, 15.0, 0.45],
            [7.2, 4.0, 0.48],
            [7.2, 10.0, 0.48],
            [7.2, 15.0, 0.48],
            [12.0, 6.2, 0.48],
            [12.0, 12.0, 0.48],
            [16.8, 3.2, 0.48],
            [16.8, 9.2, 0.48],
            [16.8, 15.0, 0.48],
            [21.8, 6.0, 0.46],
            [21.8, 11.8, 0.46],
        ],
        dtype=np.float64,
    )
    known_circles = np.hstack((known_circles, np.zeros((known_circles.shape[0], 4))))

    known_obs = np.vstack((known_circles, short_walls))

    unknown_obs = np.array(
        [
            [2.4, 6.0, 0.22],
            [2.6, 12.0, 0.22],
            [4.8, 8.0, 0.22],
            [5.6, 13.4, 0.22],
            [8.6, 6.2, 0.22],
            [8.8, 12.8, 0.22],
            [10.6, 3.6, 0.22],
            [10.8, 9.6, 0.22],
            [10.8, 15.0, 0.22],
            [13.6, 4.4, 0.22],
            [13.6, 10.0, 0.22],
            [13.6, 14.8, 0.22],
            [17.4, 5.8, 0.22],
            [17.6, 11.2, 0.22],
            [18.4, 14.6, 0.22],
            [21.0, 8.6, 0.22],
            [22.2, 10.0, 0.22],
            [22.2, 14.2, 0.22],
        ],
        dtype=np.float64,
    )

    return env_width, env_height, known_obs, unknown_obs


def build_stress_unknown_obs(layout):
    if layout == 'indoor':
        return np.array(
            [
                [5.2, 5.0, 0.24],
                [6.2, 8.8, 0.24],
                [7.2, 13.8, 0.24],
                [9.0, 7.2, 0.24],
                [11.0, 5.4, 0.24],
                [11.6, 9.8, 0.24],
                [13.8, 9.2, 0.24],
                [15.0, 11.8, 0.24],
                [17.0, 5.6, 0.24],
                [18.8, 8.8, 0.24],
                [19.8, 13.8, 0.24],
                [21.2, 9.6, 0.24],
            ],
            dtype=np.float64,
        )

    return np.array(
        [
            [4.6, 5.6, 0.24],
            [5.4, 11.0, 0.24],
            [6.8, 8.0, 0.24],
            [8.8, 4.8, 0.24],
            [9.2, 13.8, 0.24],
            [11.0, 8.0, 0.24],
            [12.6, 4.6, 0.24],
            [13.2, 13.4, 0.24],
            [15.4, 7.0, 0.24],
            [17.2, 4.8, 0.24],
            [17.8, 13.2, 0.24],
            [20.4, 9.8, 0.24],
        ],
        dtype=np.float64,
    )


def build_initial_states(num_agent):
    # These starts are specified in Vicon coordinates for consistency with the
    # real Flylab setup, then translated into the SEAMLiS map frame used by the
    # simulation environment.
    vicon_candidates = np.array(
        [
            [-1.0, 0.0,-math.pi ],
            [-1.0, -1.0, -math.pi],
            [-1.0, 0.0, -math.pi],
        ],
        dtype=np.float64,
    )
    candidates = np.array(
        [
            [*vicon_to_map_xy(x, y), yaw]
            for x, y, yaw in vicon_candidates
        ],
        dtype=np.float64,
    )
    if num_agent < 1 or num_agent > candidates.shape[0]:
        raise ValueError("num_agent must be in [1, 3] for this test scenario.")
    return [candidates[i] for i in range(num_agent)]


def get_robot_specs(num_agent, use_astar):
    robot_specs = []
    for robot_id in range(num_agent):
        if use_astar:
            robot_spec = {
                'model': 'DoubleIntegrator2D',
                'v_max': 1.35,
                'a_max': 1.5,
                'radius': 0.075,
                'sensor': 'rgbd',
                'fov_angle': 70.0,
                'cam_range': 0.8,
                'num_constraints': 20,
                'reached_threshold': 0.4,
                'min_goal_distance': 0.6,
                'nominal_k_v': 2.2,
                'nominal_k_a': 2.2,
                'unknown_obs_detection': 'fov',
                'exploration': True,
                'robot_id': robot_id,
                'visibility_violation_mode': 'point_mass',
                'visibility_violation_tolerance': 0.02,
                'deadlock_window_s': 3.0,
                'deadlock_position_eps': 0.28,
                'deadlock_speed_eps': 0.06,
                'deadlock_goal_margin': 0.9,
                'deadlock_cooldown_s': 3.5,
                'deadlock_max_recoveries': 12,
                'mpc_horizon': 10,
                'mpc_cbf_alpha1': 0.55,
                'mpc_cbf_alpha2': 0.55,
            }
        else:
            robot_spec = {
                'model': 'DoubleIntegrator2D',
                'v_max': 1.45,
                'a_max': 2.0,
                'radius': 0.20,
                'sensor': 'rgbd',
                'fov_angle': 70.0,
                'cam_range': 4.5,
                'num_constraints': 16,
                'reached_threshold': 0.45,
                'min_goal_distance': 0.8,
                'nominal_k_v': 2.4,
                'nominal_k_a': 2.3,
                'unknown_obs_detection': 'fov',
                'exploration': True,
                'robot_id': robot_id,
                'visibility_violation_mode': 'point_mass',
                'visibility_violation_tolerance': 0.02,
                'deadlock_window_s': 3.0,
                'deadlock_position_eps': 0.28,
                'deadlock_speed_eps': 0.06,
                'deadlock_goal_margin': 0.9,
                'deadlock_cooldown_s': 3.5,
                'deadlock_max_recoveries': 12,
                'mpc_horizon': 8,
                'mpc_cbf_alpha1': 0.45,
                'mpc_cbf_alpha2': 0.45,
            }
        robot_specs.append(robot_spec)
    return robot_specs


def parse_cf_ids(raw_ids):
    return [int(item.strip()) for item in raw_ids.split(',') if item.strip()]


def make_trajectory_recorder(output_path, cf_ids):
    output_path = os.path.abspath(os.path.expanduser(output_path))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    csv_file = open(output_path, 'w', newline='')
    fieldnames = [
        'step',
        'time',
        'robot_idx',
        'cf_id',
        'map_x',
        'map_y',
        'vicon_x',
        'vicon_y',
        'yaw',
        'goal_map_x',
        'goal_map_y',
    ]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    def _record(manager, step_idx, sim_time):
        for robot_idx, controller in enumerate(manager.controller_list):
            position = np.asarray(controller.robot.get_position(), dtype=float).reshape(-1)
            yaw = float(controller.robot.get_orientation())
            vicon_x, vicon_y = map_to_vicon_xy(position[0], position[1])
            goal = getattr(controller, 'goal', None)
            if goal is None:
                goal_x = ''
                goal_y = ''
            else:
                goal_vec = np.asarray(goal, dtype=float).reshape(-1)
                goal_x = goal_vec[0] if goal_vec.size > 0 else ''
                goal_y = goal_vec[1] if goal_vec.size > 1 else ''

            writer.writerow(
                {
                    'step': int(step_idx),
                    'time': f'{float(sim_time):.6f}',
                    'robot_idx': int(robot_idx),
                    'cf_id': int(cf_ids[robot_idx]),
                    'map_x': f'{position[0]:.6f}',
                    'map_y': f'{position[1]:.6f}',
                    'vicon_x': f'{vicon_x:.6f}',
                    'vicon_y': f'{vicon_y:.6f}',
                    'yaw': f'{yaw:.6f}',
                    'goal_map_x': goal_x,
                    'goal_map_y': goal_y,
                }
            )

    return output_path, csv_file, _record


def parse_args():
    parser = argparse.ArgumentParser(description='Run exploration test scenario.')
    parser.add_argument('--num_agent', type=int, default=2, help='Number of robots (supported: 1, 2, 3).')
    parser.add_argument(
        '--algo',
        type=str,
        default='frontier',
        choices=['coscan', 'frontier'],
        help='Exploration algorithm: coscan or frontier.',
    )
    parser.add_argument(
        '--layout',
        type=str,
        default='indoor',
        choices=['indoor', 'open'],
        help='Environment layout: indoor (wall-heavy) or open (obstacle-heavy).',
    )
    astar_group = parser.add_mutually_exclusive_group()
    astar_group.add_argument('--use_astar', dest='use_astar', action='store_true', help='Enable A* corridor waypoints.')
    astar_group.add_argument('--no-astar', dest='use_astar', action='store_false', help='Disable A* waypoints.')
    parser.set_defaults(use_astar=None)
    parser.add_argument(
        '--attitude',
        type=str,
        default='velocity_tracking_yaw',
        choices=['velocity_tracking_yaw', 'visibility_area', 'simple', 'visibility_raycast', 'gatekeeper', 'visibility'],
        help='Attitude controller name.',
    )
    parser.add_argument(
        '--gatekeeper_nominal',
        type=str,
        default='visibility_area',
        choices=['visibility_area', 'simple', 'velocity_tracking_yaw'],
        help='Nominal attitude controller used inside gatekeeper.',
    )
    parser.add_argument(
        '--gatekeeper_backup',
        type=str,
        default='velocity_tracking_yaw',
        choices=['velocity_tracking_yaw', 'simple'],
        help='Backup attitude controller used inside gatekeeper.',
    )
    parser.add_argument(
        '--gatekeeper_nominal_horizon',
        type=float,
        default=0.4,
        help='Gatekeeper nominal horizon [s].',
    )
    parser.add_argument(
        '--gatekeeper_backup_horizon',
        type=float,
        default=1.8,
        help='Gatekeeper backup horizon [s].',
    )
    parser.add_argument(
        '--gatekeeper_event_offset',
        type=float,
        default=0.0,
        help='Gatekeeper event offset [s].',
    )
    parser.add_argument(
        '--gatekeeper_horizon_discount',
        type=float,
        default=0.05,
        help='Gatekeeper nominal-horizon discount step [s].',
    )
    parser.add_argument(
        '--gatekeeper_validation_slack',
        type=float,
        default=0.30,
        help='Extra slack [m] for braking-distance monitor.',
    )
    parser.add_argument(
        '--gatekeeper_braking_margin',
        type=float,
        default=0.90,
        help='Extra conservative braking margin [m].',
    )
    parser.add_argument(
        '--pos_controller',
        type=str,
        default='mpc_cbf',
        choices=['cbf_qp', 'mpc_cbf'],
        help='Position controller name.',
    )
    parser.add_argument('--coverage_target', type=float, default=0.98, help='Coverage ratio target for success.')
    parser.add_argument(
        '--unknown_profile',
        type=str,
        default='default',
        choices=['default', 'stress'],
        help='Unknown-obstacle profile: default or stress (denser, harder).',
    )
    parser.add_argument('--map_resolution', type=float, default=0.16, help='Exploration map resolution [m/cell].')
    parser.add_argument('--fov_angle', type=float, default=None, help='Override robot FoV angle in degrees.')
    parser.add_argument('--cam_range', type=float, default=None, help='Override robot camera range in meters.')
    parser.add_argument('--w_max', type=float, default=None, help='Override robot max yaw rate [rad/s].')
    parser.add_argument(
        '--hide_visibility_violations',
        action='store_true',
        help='Hide visibility-violation red markers in the animation (counting is unchanged).',
    )
    parser.add_argument('--save_anim', action='store_true', help='Save animation as mp4 (rendering required).')
    parser.add_argument('--no_render', action='store_true', help='Disable live rendering (headless run).')
    parser.add_argument(
        '--output_trajectory',
        default=None,
        help='Optional CSV path for recording simulated x/y trajectory for hardware replay.',
    )
    parser.add_argument(
        '--cf_ids',
        default=None,
        help='Comma-separated Crazyflie IDs for --output_trajectory. Defaults to 1..num_agent.',
    )
    parser.add_argument('--dt', type=float, default=0.1, help='Simulation step size.')
    parser.add_argument('--tf', type=float, default=300.0, help='Simulation horizon in seconds.')
    unknown_group = parser.add_mutually_exclusive_group()
    unknown_group.add_argument('--unknown', dest='unknown', action='store_true', help='Enable unknown obstacles.')
    unknown_group.add_argument('--no-unknown', dest='unknown', action='store_false', help='Disable unknown obstacles.')
    parser.set_defaults(unknown=True)
    return parser.parse_args()


def main():
    args = parse_args()

    if args.no_render:
        import matplotlib

        matplotlib.use('Agg')

    from exploration import ExplorationManager
    from safe_control.utils import env

    layout = args.layout
    use_astar = (layout == 'indoor') if args.use_astar is None else bool(args.use_astar)

    if layout == 'indoor':
        env_width, env_height, known_obs, unknown_obs = build_indoor_exploration_env()
    else:
        env_width, env_height, known_obs, unknown_obs = build_open_exploration_env()

    if not args.unknown:
        unknown_obs = np.empty((0, 3), dtype=np.float64)
    elif args.unknown_profile == 'stress':
        unknown_obs = np.vstack((unknown_obs, build_stress_unknown_obs(layout)))

    show_animation = not args.no_render
    save_animation = args.save_anim and show_animation
    if args.no_render and args.save_anim:
        print('`--save_anim` requires rendering. Ignoring save request because `--no_render` is set.')

    x0s = build_initial_states(args.num_agent)
    robot_specs = get_robot_specs(args.num_agent, use_astar=use_astar)
    for robot_spec in robot_specs:
        if args.fov_angle is not None:
            robot_spec['fov_angle'] = float(args.fov_angle)
        if args.cam_range is not None:
            robot_spec['cam_range'] = float(args.cam_range)
        if args.w_max is not None:
            robot_spec['w_max'] = float(args.w_max)
        if args.hide_visibility_violations:
            robot_spec['show_visibility_violations'] = False
        # Keep unknown-obstacle memory persistent for each agent.
        robot_spec['unknown_obs_persistent_fov'] = True
        if args.attitude == 'gatekeeper':
            robot_spec['w_max'] = float(robot_spec.get('w_max', 1.2))
            robot_spec['visibility_violation_mode'] = 'point_mass'
            robot_spec['gatekeeper_nominal'] = args.gatekeeper_nominal
            robot_spec['gatekeeper_backup'] = args.gatekeeper_backup
            robot_spec['gatekeeper_nominal_horizon'] = float(args.gatekeeper_nominal_horizon)
            robot_spec['gatekeeper_backup_horizon'] = float(args.gatekeeper_backup_horizon)
            robot_spec['gatekeeper_event_offset'] = float(args.gatekeeper_event_offset)
            robot_spec['gatekeeper_horizon_discount'] = float(args.gatekeeper_horizon_discount)
            robot_spec['gatekeeper_validation_slack'] = float(args.gatekeeper_validation_slack)
            robot_spec['gatekeeper_braking_distance_margin'] = float(args.gatekeeper_braking_margin)
    env_handler = env.Env(
        width=env_width,
        height=env_height,
        known_obs=known_obs,
        resolution=args.map_resolution,
    )

    controller_type = {
        'pos': args.pos_controller,
        'att': args.attitude,
    }

    exploration_algorithm = 'CoScan' if args.algo == 'coscan' else 'Frontier'
    manager = ExplorationManager(
        x0s,
        robot_specs,
        controller_type,
        exploration_algorithm=exploration_algorithm,
        dt=args.dt,
        show_animation=show_animation,
        save_animation=save_animation,
        env_handler=env_handler,
        known_obs=known_obs,
        unknown_obs=unknown_obs,
        use_astar_waypoints=use_astar,
        coverage_target=args.coverage_target,
    )

    trajectory_file = None
    trajectory_output_path = None
    trajectory_callback = None
    if args.output_trajectory:
        cf_ids = parse_cf_ids(args.cf_ids) if args.cf_ids else list(range(1, args.num_agent + 1))
        if len(cf_ids) != args.num_agent:
            raise ValueError(f'--cf_ids length ({len(cf_ids)}) must match --num_agent ({args.num_agent}).')
        trajectory_output_path, trajectory_file, trajectory_callback = make_trajectory_recorder(
            args.output_trajectory,
            cf_ids,
        )

    max_steps = int(args.tf / args.dt)
    try:
        success = manager.explore(max_steps=max_steps, step_callback=trajectory_callback)
    finally:
        if trajectory_file is not None:
            trajectory_file.close()
            print(f'Recorded trajectory CSV: {trajectory_output_path}')
    violation_counts = [len(controller.robot.unsafe_points) for controller in manager.controller_list]
    print(f'Visibility violations per robot: {violation_counts} (total={sum(violation_counts)})')
    if args.attitude == 'gatekeeper':
        gk_stats = []
        for i, controller in enumerate(manager.controller_list):
            att_ctrl = getattr(controller, 'att_controller', None)
            if att_ctrl is not None and hasattr(att_ctrl, 'get_stats'):
                stats = att_ctrl.get_stats()
                gk_stats.append(
                    f"r{i}: replans={stats['replans']}, accepted={stats['accepted']}, rejected={stats['rejected']}, "
                    f"nominal_commits={stats['nominal_commits']}, "
                    f"nominal_max={stats['nominal_seconds_max']:.2f}s, nominal_avg={stats['nominal_seconds_avg_per_commit']:.2f}s"
                )
        if gk_stats:
            print('Gatekeeper nominal usage -> ' + ' | '.join(gk_stats))
    if success:
        print('Success!')
    else:
        print('Failed!')


if __name__ == '__main__':
    main()
