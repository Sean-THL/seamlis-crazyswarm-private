#!/usr/bin/env python3
import argparse
import csv
import os
import sys

import numpy as np


REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


from exploration import ExplorationManager  # noqa: E402
from examples.flylab import (  # noqa: E402
    build_indoor_exploration_env,
    build_initial_states,
    build_open_exploration_env,
    build_stress_unknown_obs,
    get_robot_specs,
    map_to_vicon_xy,
)
from safe_control.utils import env  # noqa: E402


def _parse_cf_ids(raw_ids):
    return [int(item.strip()) for item in raw_ids.split(",") if item.strip()]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run SEAMLiS simulation and record robot trajectories for hardware replay."
    )
    parser.add_argument("--cf_ids", default="6", help="Comma-separated Crazyflie IDs to assign to simulated agents.")
    parser.add_argument("--num_agent", type=int, default=None, help="Defaults to the number of --cf_ids.")
    parser.add_argument("--output", default="trajectories/seamlis_sim_trajectory.csv", help="Output CSV path.")
    parser.add_argument("--algo", choices=["frontier", "coscan"], default="frontier")
    parser.add_argument("--layout", choices=["indoor", "open"], default="indoor")
    parser.add_argument("--attitude", default="gatekeeper", choices=[
        "velocity_tracking_yaw",
        "visibility_area",
        "simple",
        "visibility_raycast",
        "gatekeeper",
        "visibility",
    ])
    parser.add_argument("--gatekeeper_nominal", default="visibility_area", choices=["visibility_area", "simple", "velocity_tracking_yaw"])
    parser.add_argument("--gatekeeper_backup", default="velocity_tracking_yaw", choices=["velocity_tracking_yaw", "simple"])
    parser.add_argument("--gatekeeper_nominal_horizon", type=float, default=0.4)
    parser.add_argument("--gatekeeper_backup_horizon", type=float, default=1.8)
    parser.add_argument("--gatekeeper_event_offset", type=float, default=0.0)
    parser.add_argument("--gatekeeper_horizon_discount", type=float, default=0.05)
    parser.add_argument("--gatekeeper_validation_slack", type=float, default=0.30)
    parser.add_argument("--gatekeeper_braking_margin", type=float, default=0.90)
    parser.add_argument("--pos_controller", choices=["cbf_qp", "mpc_cbf"], default="mpc_cbf")
    parser.add_argument("--coverage_target", type=float, default=0.98)
    parser.add_argument("--unknown_profile", choices=["default", "stress"], default="default")
    parser.add_argument("--map_resolution", type=float, default=0.16)
    parser.add_argument("--fov_angle", type=float, default=None)
    parser.add_argument("--cam_range", type=float, default=None)
    parser.add_argument("--w_max", type=float, default=None)
    parser.add_argument(
        "--hide_visibility_violations",
        action="store_true",
        help="Hide visibility-violation red markers in the animation.",
    )
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--tf", type=float, default=300.0, help="Maximum simulation horizon in seconds.")
    parser.add_argument("--show_animation", action="store_true")
    parser.add_argument("--save_animation", action="store_true")
    astar_group = parser.add_mutually_exclusive_group()
    astar_group.add_argument("--use_astar", dest="use_astar", action="store_true")
    astar_group.add_argument("--no-astar", dest="use_astar", action="store_false")
    parser.set_defaults(use_astar=None)
    unknown_group = parser.add_mutually_exclusive_group()
    unknown_group.add_argument("--unknown", dest="unknown", action="store_true")
    unknown_group.add_argument("--no-unknown", dest="unknown", action="store_false")
    parser.set_defaults(unknown=True)
    return parser.parse_args()


def _build_environment(args):
    if args.layout == "indoor":
        env_width, env_height, known_obs, unknown_obs = build_indoor_exploration_env()
    else:
        env_width, env_height, known_obs, unknown_obs = build_open_exploration_env()

    if not args.unknown:
        unknown_obs = np.empty((0, 3), dtype=np.float64)
    elif args.unknown_profile == "stress":
        unknown_obs = np.vstack((unknown_obs, build_stress_unknown_obs(args.layout)))

    return env_width, env_height, known_obs, unknown_obs


def _build_manager(args):
    use_astar = (args.layout == "indoor") if args.use_astar is None else bool(args.use_astar)
    env_width, env_height, known_obs, unknown_obs = _build_environment(args)
    x0s = build_initial_states(args.num_agent)
    robot_specs = get_robot_specs(args.num_agent, use_astar=use_astar)

    for robot_spec in robot_specs:
        if args.fov_angle is not None:
            robot_spec["fov_angle"] = float(args.fov_angle)
        if args.cam_range is not None:
            robot_spec["cam_range"] = float(args.cam_range)
        if args.w_max is not None:
            robot_spec["w_max"] = float(args.w_max)
        if args.hide_visibility_violations:
            robot_spec["show_visibility_violations"] = False
        robot_spec["unknown_obs_persistent_fov"] = True
        if args.attitude == "gatekeeper":
            robot_spec["w_max"] = float(robot_spec.get("w_max", 1.2))
            robot_spec["visibility_violation_mode"] = "point_mass"
            robot_spec["gatekeeper_nominal"] = args.gatekeeper_nominal
            robot_spec["gatekeeper_backup"] = args.gatekeeper_backup
            robot_spec["gatekeeper_nominal_horizon"] = float(args.gatekeeper_nominal_horizon)
            robot_spec["gatekeeper_backup_horizon"] = float(args.gatekeeper_backup_horizon)
            robot_spec["gatekeeper_event_offset"] = float(args.gatekeeper_event_offset)
            robot_spec["gatekeeper_horizon_discount"] = float(args.gatekeeper_horizon_discount)
            robot_spec["gatekeeper_validation_slack"] = float(args.gatekeeper_validation_slack)
            robot_spec["gatekeeper_braking_distance_margin"] = float(args.gatekeeper_braking_margin)

    env_handler = env.Env(
        width=env_width,
        height=env_height,
        known_obs=known_obs,
        resolution=args.map_resolution,
    )
    controller_type = {"pos": args.pos_controller, "att": args.attitude}
    exploration_algorithm = "CoScan" if args.algo == "coscan" else "Frontier"
    return ExplorationManager(
        x0s,
        robot_specs,
        controller_type,
        exploration_algorithm=exploration_algorithm,
        dt=args.dt,
        show_animation=bool(args.show_animation or args.save_animation),
        save_animation=bool(args.save_animation),
        env_handler=env_handler,
        known_obs=known_obs,
        unknown_obs=unknown_obs,
        use_astar_waypoints=use_astar,
        coverage_target=args.coverage_target,
    )


def _record_snapshot(writer, manager, cf_ids, step_idx, sim_time):
    for robot_idx, controller in enumerate(manager.controller_list):
        position = np.asarray(controller.robot.get_position(), dtype=float).reshape(-1)
        yaw = float(controller.robot.get_orientation())
        vicon_x, vicon_y = map_to_vicon_xy(position[0], position[1])
        goal = getattr(controller, "goal", None)
        if goal is None:
            goal_x = ""
            goal_y = ""
        else:
            goal_vec = np.asarray(goal, dtype=float).reshape(-1)
            goal_x = goal_vec[0] if goal_vec.size > 0 else ""
            goal_y = goal_vec[1] if goal_vec.size > 1 else ""

        writer.writerow(
            {
                "step": step_idx,
                "time": f"{sim_time:.6f}",
                "robot_idx": robot_idx,
                "cf_id": cf_ids[robot_idx],
                "map_x": f"{position[0]:.6f}",
                "map_y": f"{position[1]:.6f}",
                "vicon_x": f"{vicon_x:.6f}",
                "vicon_y": f"{vicon_y:.6f}",
                "yaw": f"{yaw:.6f}",
                "goal_map_x": goal_x,
                "goal_map_y": goal_y,
            }
        )


def run_recording(args):
    cf_ids = _parse_cf_ids(args.cf_ids)
    if args.num_agent is None:
        args.num_agent = len(cf_ids)
    if args.num_agent != len(cf_ids):
        raise ValueError(f"--num_agent={args.num_agent} must match len(--cf_ids)={len(cf_ids)}.")

    manager = _build_manager(args)
    max_steps = int(args.tf / args.dt)
    output_path = os.path.abspath(os.path.expanduser(args.output))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    fieldnames = [
        "step",
        "time",
        "robot_idx",
        "cf_id",
        "map_x",
        "map_y",
        "vicon_x",
        "vicon_y",
        "yaw",
        "goal_map_x",
        "goal_map_y",
    ]

    with open(output_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        manager.failed = False
        manager.frontiers = manager.get_frontiers()
        manager.last_coverage_ratio = manager.get_coverage_ratio()
        manager.update_all_goals()
        _record_snapshot(writer, manager, cf_ids, 0, 0.0)

        success = False
        termination_reason = "max_steps"
        for step_idx in range(1, max_steps + 1):
            robots_reached_goals = manager.move_robots()
            sim_time = step_idx * args.dt
            _record_snapshot(writer, manager, cf_ids, step_idx, sim_time)

            if manager.failed:
                termination_reason = "collision_or_infeasible"
                break

            refresh_map = any(robots_reached_goals) or (step_idx % 15 == 0)
            if refresh_map:
                manager.frontiers = manager.get_frontiers()
                manager.last_coverage_ratio = manager.get_coverage_ratio()
                if manager.last_coverage_ratio >= args.coverage_target:
                    success = True
                    termination_reason = "coverage_target_reached"
                    break
                if manager.exploration_complete():
                    termination_reason = "frontiers_exhausted"
                    break

            if any(robots_reached_goals):
                manager.update_goals_for_completed(robots_reached_goals)

            if manager.show_animation:
                manager.update_visualization()

    if manager.save_animation:
        manager._finalize_animation()

    print(f"Recorded trajectory CSV: {output_path}")
    print(f"success={success} termination={termination_reason} coverage={manager.last_coverage_ratio:.3f}")
    return success


def main():
    args = parse_args()
    run_recording(args)


if __name__ == "__main__":
    main()
