#!/usr/bin/env python3
import argparse
import os
import sys

import numpy as np
import rospy
from crazyswarm.msg import Position
from shapely.geometry import Point, Polygon
from std_msgs.msg import Bool


REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


from cf_state_subscriber import CfStateTracker, angle_normalize, get_cf_ids  # noqa: E402
from exploration import ExplorationManager  # noqa: E402
from examples.flylab import (  # noqa: E402
    build_indoor_exploration_env,
    build_open_exploration_env,
    build_stress_unknown_obs,
    build_initial_states,
    get_robot_specs,
    map_to_vicon_xy,
    vicon_to_map_xy,
)
from safe_control.utils import env  # noqa: E402


DONE_TOPIC = "/seamlis/exploration_done"
DIRECT_CMD_TOPIC_SUFFIX = "/cmd_position"
DEFAULT_FLIGHT_Z = 1.0
MAX_CONTROL_ACCEL = 1.5
MAX_MEASURED_VELOCITY = 0.8
MAX_REFERENCE_STEP_XY = 0.12
MAX_YAW_STEP = 0.15


def parse_args():
    parser = argparse.ArgumentParser(description="Run SEAMLiS exploration with live Crazyswarm/VICON state.")
    parser.add_argument("--num_agent", type=int, default=None, help="Number of robots. Defaults to ROS crazyflies count.")
    parser.add_argument(
        "--algo",
        type=str,
        default="frontier",
        choices=["coscan", "frontier"],
        help="Exploration algorithm.",
    )
    parser.add_argument(
        "--layout",
        type=str,
        default="indoor",
        choices=["indoor", "open"],
        help="Environment layout.",
    )
    astar_group = parser.add_mutually_exclusive_group()
    astar_group.add_argument("--use_astar", dest="use_astar", action="store_true", help="Enable A* waypoints.")
    astar_group.add_argument("--no-astar", dest="use_astar", action="store_false", help="Disable A* waypoints.")
    parser.set_defaults(use_astar=None)
    parser.add_argument(
        "--attitude",
        type=str,
        default="gatekeeper",
        choices=["velocity_tracking_yaw", "visibility_area", "simple", "visibility_raycast", "gatekeeper", "visibility"],
        help="Attitude controller.",
    )
    parser.add_argument(
        "--gatekeeper_nominal",
        type=str,
        default="visibility_area",
        choices=["visibility_area", "simple", "velocity_tracking_yaw"],
        help="Gatekeeper nominal attitude controller.",
    )
    parser.add_argument(
        "--gatekeeper_backup",
        type=str,
        default="velocity_tracking_yaw",
        choices=["velocity_tracking_yaw", "simple"],
        help="Gatekeeper backup attitude controller.",
    )
    parser.add_argument("--gatekeeper_nominal_horizon", type=float, default=0.4)
    parser.add_argument("--gatekeeper_backup_horizon", type=float, default=1.8)
    parser.add_argument("--gatekeeper_event_offset", type=float, default=0.0)
    parser.add_argument("--gatekeeper_horizon_discount", type=float, default=0.05)
    parser.add_argument("--gatekeeper_validation_slack", type=float, default=0.30)
    parser.add_argument("--gatekeeper_braking_margin", type=float, default=0.90)
    parser.add_argument(
        "--pos_controller",
        type=str,
        default="mpc_cbf",
        choices=["cbf_qp", "mpc_cbf"],
        help="Position controller.",
    )
    parser.add_argument("--coverage_target", type=float, default=0.98)
    parser.add_argument(
        "--unknown_profile",
        type=str,
        default="default",
        choices=["default", "stress"],
        help="Unknown obstacle profile.",
    )
    parser.add_argument("--map_resolution", type=float, default=0.16)
    parser.add_argument("--fov_angle", type=float, default=None)
    parser.add_argument("--cam_range", type=float, default=None)
    parser.add_argument("--w_max", type=float, default=None)
    parser.add_argument("--dt", type=float, default=0.1, help="SEAMLiS controller dt.")
    parser.add_argument("--fake_dt", type=float, default=0.35, help="Prediction horizon used to convert control to next position.")
    parser.add_argument("--command_rate_hz", type=float, default=15.0, help="Reference publish rate.")
    parser.add_argument("--velocity_alpha", type=float, default=0.5, help="Low-pass blend for VICON velocity estimation.")
    parser.add_argument("--show_animation", action="store_true", help="Show live SEAMLiS visualization.")
    parser.add_argument("--save_animation", action="store_true", help="Save live SEAMLiS visualization frames/video.")
    parser.add_argument("--visualization_rate_hz", type=float, default=5.0, help="Maximum live visualization refresh rate.")
    parser.add_argument(
        "--publish_current_yaw",
        dest="publish_current_yaw",
        action="store_true",
        help="Send the measured yaw directly in the position command.",
    )
    parser.add_argument(
        "--predict_yaw",
        dest="publish_current_yaw",
        action="store_false",
        help="Project yaw forward using the SEAMLiS attitude output.",
    )
    parser.set_defaults(publish_current_yaw=True)
    unknown_group = parser.add_mutually_exclusive_group()
    unknown_group.add_argument("--unknown", dest="unknown", action="store_true", help="Enable unknown obstacles.")
    unknown_group.add_argument("--no-unknown", dest="unknown", action="store_false", help="Disable unknown obstacles.")
    parser.set_defaults(unknown=True)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


class ExplorationCrazyswarm:
    def __init__(self, args):
        self.args = args
        self.cf_ids = get_cf_ids()
        if not self.cf_ids:
            raise RuntimeError("No Crazyflie IDs found in ROS param 'crazyflies'. Launch crazyswarm first.")

        if self.args.num_agent is None:
            self.args.num_agent = len(self.cf_ids)
        if self.args.num_agent != len(self.cf_ids):
            raise ValueError(
                f"--num_agent={self.args.num_agent} does not match ROS crazyflies count={len(self.cf_ids)}."
            )

        self.cf_id_to_robot_idx = {cf_id: idx for idx, cf_id in enumerate(self.cf_ids)}
        self.use_astar = (self.args.layout == "indoor") if self.args.use_astar is None else bool(self.args.use_astar)
        self.state_tracker = CfStateTracker(cf_ids=self.cf_ids, velocity_alpha=self.args.velocity_alpha)
        self.manager = self._build_manager()
        self.goals_initialized_from_live_state = False
        self.live_state_initialized = False
        self._last_command_log_time = rospy.Time(0)
        self._last_visualization_time = rospy.Time(0)
        self._animation_finalized = False
        self.command_publishers = {
            cf_id: rospy.Publisher(f"/cf{cf_id}{DIRECT_CMD_TOPIC_SUFFIX}", Position, queue_size=10)
            for cf_id in self.cf_ids
        }
        self.done_publisher = rospy.Publisher(DONE_TOPIC, Bool, queue_size=1, latch=False)

        rospy.loginfo(
            "ExplorationCrazyswarm configured for cf_ids=%s layout=%s algo=%s fake_dt=%.3f",
            self.cf_ids,
            self.args.layout,
            self.args.algo,
            self.args.fake_dt,
        )
        rospy.loginfo(
            "Publishing live commands directly to /cf<ID>%s at fixed z=%.3f m.",
            DIRECT_CMD_TOPIC_SUFFIX,
            DEFAULT_FLIGHT_Z,
        )

    def _build_environment(self):
        if self.args.layout == "indoor":
            env_width, env_height, known_obs, unknown_obs = build_indoor_exploration_env()
        else:
            env_width, env_height, known_obs, unknown_obs = build_open_exploration_env()

        if not self.args.unknown:
            unknown_obs = np.empty((0, 3), dtype=np.float64)
        elif self.args.unknown_profile == "stress":
            unknown_obs = np.vstack((unknown_obs, build_stress_unknown_obs(self.args.layout)))

        return env_width, env_height, known_obs, unknown_obs

    def _build_manager(self):
        env_width, env_height, known_obs, unknown_obs = self._build_environment()
        x0s = build_initial_states(self.args.num_agent)
        robot_specs = get_robot_specs(self.args.num_agent, use_astar=self.use_astar)

        for robot_spec in robot_specs:
            if self.args.fov_angle is not None:
                robot_spec["fov_angle"] = float(self.args.fov_angle)
            if self.args.cam_range is not None:
                robot_spec["cam_range"] = float(self.args.cam_range)
            if self.args.w_max is not None:
                robot_spec["w_max"] = float(self.args.w_max)
            robot_spec["unknown_obs_persistent_fov"] = True
            if self.args.attitude == "gatekeeper":
                robot_spec["w_max"] = float(robot_spec.get("w_max", 1.2))
                robot_spec["visibility_violation_mode"] = "point_mass"
                robot_spec["gatekeeper_nominal"] = self.args.gatekeeper_nominal
                robot_spec["gatekeeper_backup"] = self.args.gatekeeper_backup
                robot_spec["gatekeeper_nominal_horizon"] = float(self.args.gatekeeper_nominal_horizon)
                robot_spec["gatekeeper_backup_horizon"] = float(self.args.gatekeeper_backup_horizon)
                robot_spec["gatekeeper_event_offset"] = float(self.args.gatekeeper_event_offset)
                robot_spec["gatekeeper_horizon_discount"] = float(self.args.gatekeeper_horizon_discount)
                robot_spec["gatekeeper_validation_slack"] = float(self.args.gatekeeper_validation_slack)
                robot_spec["gatekeeper_braking_distance_margin"] = float(self.args.gatekeeper_braking_margin)

        env_handler = env.Env(
            width=env_width,
            height=env_height,
            known_obs=known_obs,
            resolution=self.args.map_resolution,
        )

        controller_type = {
            "pos": self.args.pos_controller,
            "att": self.args.attitude,
        }
        exploration_algorithm = "CoScan" if self.args.algo == "coscan" else "Frontier"

        manager = ExplorationManager(
            x0s,
            robot_specs,
            controller_type,
            exploration_algorithm=exploration_algorithm,
            dt=self.args.dt,
            show_animation=bool(self.args.show_animation or self.args.save_animation),
            save_animation=bool(self.args.save_animation),
            env_handler=env_handler,
            known_obs=known_obs,
            unknown_obs=unknown_obs,
            use_astar_waypoints=self.use_astar,
            coverage_target=self.args.coverage_target,
        )
        return manager

    def _maybe_update_visualization(self, force=False):
        if not self.manager.show_animation:
            return

        now = rospy.Time.now()
        min_period = 1.0 / max(float(self.args.visualization_rate_hz), 1e-3)
        if not force and (now - self._last_visualization_time).to_sec() < min_period:
            return

        self._last_visualization_time = now
        self.manager.update_visualization()

    def _finalize_animation(self):
        if self._animation_finalized:
            return
        self._animation_finalized = True
        if self.manager.save_animation:
            self.manager._finalize_animation()

    def _initialize_goals_from_live_state(self):
        self.manager.frontiers = self.manager.get_frontiers()
        self.manager.update_all_goals()
        self.manager.last_coverage_ratio = self.manager.get_coverage_ratio()
        self.goals_initialized_from_live_state = True
        frontier_count = len(getattr(self.manager.frontiers, "coords", []))
        goals = getattr(self.manager, "global_goals", None)
        waypoint_counts = [
            len(getattr(controller, "waypoints", []))
            for controller in self.manager.controller_list
        ]
        rospy.loginfo(
            "Initialized SEAMLiS goals after live VICON state synchronization: "
            "frontiers=%s goals=%s waypoint_counts=%s",
            frontier_count,
            goals,
            waypoint_counts,
        )
        if goals is None or all(count == 0 for count in waypoint_counts):
            rospy.logwarn(
                "SEAMLiS initialized without active goals. "
                "frontiers=%s coverage=%.3f waypoint_counts=%s",
                frontier_count,
                float(getattr(self.manager, "last_coverage_ratio", 0.0)),
                waypoint_counts,
            )

    def _validate_live_start_configuration(self):
        for robot_idx, controller in enumerate(self.manager.controller_list):
            robot_pos = np.asarray(controller.robot.get_position(), dtype=float).reshape(-1)
            robot_radius = float(controller.robot.robot_radius)

            for obs_idx, obs in enumerate(np.asarray(self.manager.unknown_obs, dtype=float)):
                if obs.shape[0] < 3:
                    continue
                distance = float(np.linalg.norm(robot_pos[:2] - obs[:2]))
                threshold = float(robot_radius + obs[2])
                if distance < threshold:
                    vicon_x, vicon_y = map_to_vicon_xy(robot_pos[0], robot_pos[1])
                    obs_vicon_x, obs_vicon_y = map_to_vicon_xy(obs[0], obs[1])
                    rospy.logerr(
                        "Preflight collision: cf%s starts inside unknown obstacle %d. "
                        "robot_map=(%.3f, %.3f) robot_vicon=(%.3f, %.3f) "
                        "obs_map=(%.3f, %.3f, r=%.3f) obs_vicon=(%.3f, %.3f) "
                        "distance=%.3f threshold=%.3f",
                        self.cf_ids[robot_idx],
                        obs_idx,
                        robot_pos[0],
                        robot_pos[1],
                        vicon_x,
                        vicon_y,
                        obs[0],
                        obs[1],
                        obs[2],
                        obs_vicon_x,
                        obs_vicon_y,
                        distance,
                        threshold,
                    )
                    return False
        return True

    def _overwrite_robot_state(self, robot_idx, measured_state):
        controller = self.manager.controller_list[robot_idx]
        robot = controller.robot

        map_x, map_y = vicon_to_map_xy(measured_state.x, measured_state.y)
        robot.X[0, 0] = map_x
        robot.X[1, 0] = map_y
        if robot.X.shape[0] >= 4:
            robot.X[2, 0] = measured_state.vx
            robot.X[3, 0] = measured_state.vy
        robot.yaw = angle_normalize(measured_state.yaw)

    def _reset_live_exploration_context(self, robot_idx, measured_state):
        controller = self.manager.controller_list[robot_idx]
        robot = controller.robot
        map_x, map_y = vicon_to_map_xy(measured_state.x, measured_state.y)

        robot.detected_unknown_obs_memory = np.empty((0, 7))
        robot.positions = []
        robot.safety_area = Polygon()
        robot.sensing_footprints = Polygon()

        init_robot_position = Point(map_x, map_y).buffer(robot.robot_radius * 2.0)
        if robot.robot_spec.get("exploration", False):
            robot.sensing_footprints = robot.sensing_footprints.union(init_robot_position).buffer(robot.robot_radius * 2.0)
        else:
            robot.sensing_footprints = robot.sensing_footprints.union(init_robot_position)
            if robot.robot_spec.get("sensor") == "rgbd":
                robot.update_sensing_footprints()

        n_pos = 3 if robot.robot_spec["model"] in ["Quad3D"] else 2
        controller.waypoints = np.empty((0, n_pos), dtype=float)
        controller.current_goal_index = 0
        controller.goal = None
        controller.state_machine = "idle"
        controller.u_att = None

        rospy.loginfo(
            "Reset live exploration context for cf%s at map=(%.3f, %.3f) from VICON=(%.3f, %.3f).",
            self.cf_ids[robot_idx],
            map_x,
            map_y,
            measured_state.x,
            measured_state.y,
        )

    def _project_reference(self, measured_state, u_pos, u_att):
        u_vec = np.asarray(u_pos, dtype=float).reshape(-1)
        ax = float(u_vec[0]) if u_vec.size > 0 else 0.0
        ay = float(u_vec[1]) if u_vec.size > 1 else 0.0
        ax = float(np.clip(ax, -MAX_CONTROL_ACCEL, MAX_CONTROL_ACCEL))
        ay = float(np.clip(ay, -MAX_CONTROL_ACCEL, MAX_CONTROL_ACCEL))
        vx = float(np.clip(measured_state.vx, -MAX_MEASURED_VELOCITY, MAX_MEASURED_VELOCITY))
        vy = float(np.clip(measured_state.vy, -MAX_MEASURED_VELOCITY, MAX_MEASURED_VELOCITY))

        map_x, map_y = vicon_to_map_xy(measured_state.x, measured_state.y)
        dx = vx * self.args.fake_dt + 0.5 * ax * (self.args.fake_dt ** 2)
        dy = vy * self.args.fake_dt + 0.5 * ay * (self.args.fake_dt ** 2)
        step_norm = float(np.hypot(dx, dy))
        if step_norm > MAX_REFERENCE_STEP_XY:
            scale = MAX_REFERENCE_STEP_XY / step_norm
            dx *= scale
            dy *= scale
        x_next = map_x + dx
        y_next = map_y + dy
        z_next = float(DEFAULT_FLIGHT_Z)

        if self.args.publish_current_yaw:
            yaw_next = angle_normalize(measured_state.yaw)
        else:
            yaw_rate = 0.0
            if u_att is not None:
                u_att_vec = np.asarray(u_att, dtype=float).reshape(-1)
                if u_att_vec.size > 0:
                    yaw_rate = float(u_att_vec[0])
            yaw_step = float(np.clip(yaw_rate * self.args.fake_dt, -MAX_YAW_STEP, MAX_YAW_STEP))
            yaw_next = angle_normalize(measured_state.yaw + yaw_step)

        return np.array([x_next, y_next, z_next], dtype=float), yaw_next

    def _publish_reference(self, cf_id, position_cmd, yaw_cmd):
        vicon_x, vicon_y = map_to_vicon_xy(position_cmd[0], position_cmd[1])
        msg = Position()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.x = float(vicon_x)
        msg.y = float(vicon_y)
        msg.z = float(position_cmd[2])
        msg.yaw = float(yaw_cmd)
        self.command_publishers[cf_id].publish(msg)

    def _publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_publisher.publish(msg)
        rospy.loginfo("Published exploration done signal on %s", DONE_TOPIC)

    def _log_command_debug(self, cf_id, controller, measured_state, pos_cmd, yaw_cmd):
        now = rospy.Time.now()
        if (now - self._last_command_log_time).to_sec() < 1.0:
            return
        self._last_command_log_time = now
        waypoints = getattr(controller, "waypoints", [])
        goal = getattr(controller, "goal", None)
        u_pos = getattr(controller, "u_pos", None)
        u_att = getattr(controller, "u_att", None)
        vicon_x, vicon_y = map_to_vicon_xy(pos_cmd[0], pos_cmd[1])
        rospy.loginfo(
            "SEAMLiS command cf%s: measured=(%.3f, %.3f, yaw=%.3f) "
            "goal=%s waypoints=%s u_pos=%s u_att=%s ref=(%.3f, %.3f, z=%.3f, yaw=%.3f)",
            cf_id,
            measured_state.x,
            measured_state.y,
            measured_state.yaw,
            goal,
            len(waypoints),
            u_pos,
            u_att,
            vicon_x,
            vicon_y,
            pos_cmd[2],
            yaw_cmd,
        )

    def run(self):
        rate = rospy.Rate(self.args.command_rate_hz)
        cycle_count = 0

        while not rospy.is_shutdown():
            if not self.state_tracker.all_states_ready():
                rate.sleep()
                continue

            synced_states = self.state_tracker.get_all_states()
            for cf_id in self.cf_ids:
                self._overwrite_robot_state(self.cf_id_to_robot_idx[cf_id], synced_states[cf_id])

            if not self.live_state_initialized:
                for cf_id in self.cf_ids:
                    self._reset_live_exploration_context(
                        self.cf_id_to_robot_idx[cf_id],
                        synced_states[cf_id],
                    )
                self.live_state_initialized = True

            if not self.goals_initialized_from_live_state:
                self._initialize_goals_from_live_state()
                if not self._validate_live_start_configuration():
                    rospy.logerr(
                        "ExplorationCrazyswarm stopped during preflight validation. "
                        "Move the drone(s) away from the configured obstacle or update flylab.py."
                    )
                    self._finalize_animation()
                    return
                self._maybe_update_visualization(force=True)

            robots_reached_goals = self.manager.move_robots()
            if self.manager.failed:
                rospy.logerr("ExplorationCrazyswarm stopped because SEAMLiS reported collision/infeasible.")
                self._finalize_animation()
                return

            cycle_count += 1
            if any(robots_reached_goals) or (cycle_count % 15 == 0):
                self.manager.frontiers = self.manager.get_frontiers()
                self.manager.last_coverage_ratio = self.manager.get_coverage_ratio()
                if self.manager.last_coverage_ratio >= self.args.coverage_target:
                    rospy.loginfo("Coverage target reached: %.3f", self.manager.last_coverage_ratio)
                    self._publish_done()
                    self._finalize_animation()
                    return

            if any(robots_reached_goals):
                self.manager.update_goals_for_completed(robots_reached_goals)
                self._maybe_update_visualization(force=True)

            for cf_id in self.cf_ids:
                robot_idx = self.cf_id_to_robot_idx[cf_id]
                controller = self.manager.controller_list[robot_idx]
                measured_state = synced_states[cf_id]
                u_pos = controller.u_pos if hasattr(controller, "u_pos") else controller.robot.U
                pos_cmd, yaw_cmd = self._project_reference(measured_state, u_pos, controller.u_att)
                self._publish_reference(cf_id, pos_cmd, yaw_cmd)
                self._log_command_debug(cf_id, controller, measured_state, pos_cmd, yaw_cmd)

            coverage = getattr(self.manager, "last_coverage_ratio", 0.0)
            rospy.loginfo_throttle(1.0, "Exploration loop active. coverage=%.3f", coverage)
            self._maybe_update_visualization()
            rate.sleep()


def main():
    rospy.init_node("exploration_crazyswarm", anonymous=True)
    args = parse_args()
    node = ExplorationCrazyswarm(args)
    try:
        node.run()
    finally:
        node._finalize_animation()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
