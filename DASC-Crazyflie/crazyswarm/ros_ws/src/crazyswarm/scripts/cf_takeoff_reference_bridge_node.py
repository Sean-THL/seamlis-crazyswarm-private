#!/usr/bin/env python3
import argparse
import math
import threading

import rospy
from crazyswarm.msg import Position
from crazyswarm.srv import Land, Takeoff
from std_msgs.msg import Bool


DEFAULT_TAKEOFF_HEIGHT = 0.4
DEFAULT_SETTLE_DURATION = 2.0
MAX_CMD_STEP_XY = 0.01
MAX_CMD_YAW_STEP = 0.03


def get_cf_ids():
    crazyflies = rospy.get_param("crazyflies", [])
    cf_ids = []
    for cf in crazyflies:
        try:
            cf_ids.append(int(cf["id"]))
        except (KeyError, TypeError, ValueError):
            rospy.logwarn("Skipping malformed crazyflies entry: %s", cf)
    return cf_ids


def parse_cf_ids(value):
    if not value:
        return []
    return [int(part.strip()) for part in value.split(",") if part.strip()]


class FixedHeightReferenceBridge:
    """Take off to a fixed height, then forward x/y/yaw references with fixed z."""

    def __init__(
        self,
        cf_ids,
        height,
        takeoff_duration,
        rate_hz,
        reference_timeout,
        state_timeout,
        done_topic,
        land_on_done,
        landing_height,
        landing_duration,
    ):
        if not cf_ids:
            raise RuntimeError("No Crazyflie IDs configured. Launch crazyswarm first or pass --cf_ids.")

        self.cf_ids = cf_ids
        self.height = float(height)
        self.takeoff_duration = float(takeoff_duration)
        self.reference_timeout = float(reference_timeout)
        self.state_timeout = float(state_timeout)
        self.done_topic = str(done_topic)
        self.land_on_done = bool(land_on_done)
        self.landing_height = float(landing_height)
        self.landing_duration = float(landing_duration)
        self.command_enable_time = None
        self.lock = threading.Lock()
        self.landing_requested = threading.Event()
        self.landing_started = False
        self.latest_reference = {cf_id: None for cf_id in self.cf_ids}
        self.latest_reference_time = {cf_id: None for cf_id in self.cf_ids}
        self.latest_state = {cf_id: None for cf_id in self.cf_ids}
        self.last_command = {cf_id: None for cf_id in self.cf_ids}
        self.started_publishing = {cf_id: False for cf_id in self.cf_ids}

        self.cmd_publishers = {
            cf_id: rospy.Publisher(f"/cf{cf_id}/cmd_position", Position, queue_size=1)
            for cf_id in self.cf_ids
        }
        self.reference_subscribers = [
            rospy.Subscriber(
                f"/cf{cf_id}/reference",
                Position,
                self._make_reference_callback(cf_id),
                queue_size=1,
            )
            for cf_id in self.cf_ids
        ]
        self.state_subscribers = [
            rospy.Subscriber(
                f"/cf{cf_id}/state",
                Position,
                self._make_state_callback(cf_id),
                queue_size=1,
            )
            for cf_id in self.cf_ids
        ]
        self.done_subscriber = rospy.Subscriber(self.done_topic, Bool, self._done_callback, queue_size=1)
        self.rate = rospy.Rate(rate_hz)

        rospy.loginfo(
            "Fixed-height reference bridge configured for CF IDs %s at z=%.3f m",
            ", ".join(str(cf_id) for cf_id in self.cf_ids),
            self.height,
        )
        rospy.loginfo(
            "Final command limits: max_xy_step=%.3f m per tick, max_yaw_step=%.3f rad per tick.",
            MAX_CMD_STEP_XY,
            MAX_CMD_YAW_STEP,
        )
        rospy.loginfo(
            "Holding x/y/yaw during takeoff settle for %.1f s before forwarding /cf<ID>/reference.",
            DEFAULT_SETTLE_DURATION,
        )
        rospy.loginfo(
            "Subscribing /cf<ID>/reference and publishing /cf<ID>/cmd_position with fixed z."
        )
        rospy.loginfo(
            "Landing on done signal is %s; listening on %s.",
            "enabled" if self.land_on_done else "disabled",
            self.done_topic,
        )

    def _done_callback(self, msg):
        if msg.data:
            rospy.loginfo("Received exploration done signal on %s.", self.done_topic)
            if self.land_on_done:
                self.landing_requested.set()

    def _make_reference_callback(self, cf_id):
        def callback(msg):
            with self.lock:
                self.latest_reference[cf_id] = msg
                self.latest_reference_time[cf_id] = rospy.Time.now()

        return callback

    def _make_state_callback(self, cf_id):
        def callback(msg):
            with self.lock:
                self.latest_state[cf_id] = msg

        return callback

    def _limit_command(self, cf_id, reference, now):
        previous = self.last_command[cf_id] or self.latest_state[cf_id]
        cmd = Position()
        cmd.header.stamp = now
        cmd.header.frame_id = reference.header.frame_id or "world"
        cmd.z = self.height

        if previous is None:
            cmd.x = float(reference.x)
            cmd.y = float(reference.y)
            cmd.yaw = float(reference.yaw)
            self.last_command[cf_id] = cmd
            return cmd

        dx = float(reference.x) - float(previous.x)
        dy = float(reference.y) - float(previous.y)
        step_norm = math.hypot(dx, dy)
        if step_norm > MAX_CMD_STEP_XY:
            scale = MAX_CMD_STEP_XY / step_norm
            dx *= scale
            dy *= scale

        yaw_error = ((float(reference.yaw) - float(previous.yaw) + math.pi) % (2.0 * math.pi)) - math.pi
        yaw_error = max(min(yaw_error, MAX_CMD_YAW_STEP), -MAX_CMD_YAW_STEP)

        cmd.x = float(previous.x) + dx
        cmd.y = float(previous.y) + dy
        cmd.yaw = ((float(previous.yaw) + yaw_error + math.pi) % (2.0 * math.pi)) - math.pi
        self.last_command[cf_id] = cmd
        return cmd

    def _build_hold_command(self, cf_id, now):
        state = self.latest_state[cf_id] or self.last_command[cf_id]
        if state is None:
            return None
        cmd = Position()
        cmd.header.stamp = now
        cmd.header.frame_id = state.header.frame_id or "world"
        cmd.x = float(state.x)
        cmd.y = float(state.y)
        cmd.z = self.height
        cmd.yaw = float(state.yaw)
        self.last_command[cf_id] = cmd
        return cmd

    def takeoff_all(self):
        if self.takeoff_duration <= 0.0:
            rospy.loginfo("Skipping takeoff because --takeoff_duration <= 0.")
            return

        self._wait_for_state_feedback()

        rospy.loginfo(
            "Calling takeoff for %s to z=%.3f m over %.3f s.",
            ", ".join(f"cf{cf_id}" for cf_id in self.cf_ids),
            self.height,
            self.takeoff_duration,
        )

        duration = rospy.Duration.from_sec(self.takeoff_duration)
        for cf_id in self.cf_ids:
            service_name = f"/cf{cf_id}/takeoff"
            rospy.wait_for_service(service_name)
            takeoff = rospy.ServiceProxy(service_name, Takeoff)
            takeoff(groupMask=0, height=self.height, duration=duration)

        rospy.sleep(self.takeoff_duration)
        self.command_enable_time = rospy.Time.now() + rospy.Duration.from_sec(DEFAULT_SETTLE_DURATION)
        rospy.loginfo(
            "Takeoff phase complete. Holding current x/y/yaw until %.3f before accepting /cf<ID>/reference commands.",
            self.command_enable_time.to_sec(),
        )

    def land_all(self):
        if self.landing_started:
            return
        self.landing_started = True

        rospy.loginfo(
            "Calling land for %s to z=%.3f m over %.3f s.",
            ", ".join(f"cf{cf_id}" for cf_id in self.cf_ids),
            self.landing_height,
            self.landing_duration,
        )

        duration = rospy.Duration.from_sec(self.landing_duration)
        for cf_id in self.cf_ids:
            service_name = f"/cf{cf_id}/land"
            rospy.wait_for_service(service_name)
            land = rospy.ServiceProxy(service_name, Land)
            land(groupMask=0, height=self.landing_height, duration=duration)

        rospy.sleep(max(self.landing_duration, 0.0))
        rospy.loginfo("Landing command complete.")

    def _wait_for_state_feedback(self):
        if self.state_timeout <= 0.0:
            rospy.logwarn("Skipping state-feedback wait because --state_timeout <= 0.")
            return

        for cf_id in self.cf_ids:
            topic_name = f"/cf{cf_id}/state"
            rospy.loginfo(
                "Waiting up to %.1f s for state feedback on %s before takeoff.",
                self.state_timeout,
                topic_name,
            )
            try:
                state = rospy.wait_for_message(topic_name, Position, timeout=self.state_timeout)
                rospy.loginfo(
                    "State feedback ready for cf%s: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                    cf_id,
                    state.x,
                    state.y,
                    state.z,
                    state.yaw,
                )
            except rospy.ROSException as exc:
                raise RuntimeError(
                    f"No state feedback on {topic_name} before takeoff. "
                    "Fix Vicon/TF/cf_state_publisher first, or pass --state_timeout 0 to bypass this safety gate."
                ) from exc

    def spin(self):
        while not rospy.is_shutdown():
            if self.landing_requested.is_set():
                self.land_all()
                return

            now = rospy.Time.now()
            with self.lock:
                refs = dict(self.latest_reference)
                ref_times = dict(self.latest_reference_time)
                states = dict(self.latest_state)

            settle_active = self.command_enable_time is not None and now < self.command_enable_time

            if settle_active:
                for cf_id in self.cf_ids:
                    cmd = self._build_hold_command(cf_id, now)
                    if cmd is None:
                        continue
                    self.cmd_publishers[cf_id].publish(cmd)
                rospy.loginfo_throttle(
                    1.0,
                    "Takeoff settle active for %.2f more seconds; holding x/y/yaw.",
                    max((self.command_enable_time - now).to_sec(), 0.0),
                )
                self.rate.sleep()
                continue

            for cf_id, reference in refs.items():
                if reference is None:
                    rospy.logwarn_throttle(
                        5.0,
                        "No /cf%s/reference received yet; not publishing /cf%s/cmd_position.",
                        cf_id,
                        cf_id,
                    )
                    continue

                age = (now - ref_times[cf_id]).to_sec() if ref_times[cf_id] else math.inf
                if age > self.reference_timeout:
                    rospy.logwarn_throttle(
                        2.0,
                        "/cf%s/reference is stale by %.3f s; holding last x/y/yaw at fixed z.",
                        cf_id,
                        age,
                    )

                if states.get(cf_id) is None:
                    rospy.logwarn_throttle(
                        2.0,
                        "No /cf%s/state available while forwarding references; skipping /cf%s/cmd_position.",
                        cf_id,
                        cf_id,
                    )
                    continue

                cmd = self._limit_command(cf_id, reference, now)
                self.cmd_publishers[cf_id].publish(cmd)

                if not self.started_publishing[cf_id]:
                    rospy.loginfo(
                        "Started publishing /cf%s/cmd_position: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                        cf_id,
                        cmd.x,
                        cmd.y,
                        cmd.z,
                        cmd.yaw,
                    )
                    self.started_publishing[cf_id] = True

            self.rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Take off Crazyflies to a fixed height, then forward x/y/yaw references with z held constant."
    )
    parser.add_argument("--cf_ids", default="", help="Comma-separated CF IDs. Defaults to ROS param crazyflies.")
    parser.add_argument("--height", type=float, default=DEFAULT_TAKEOFF_HEIGHT, help="Fixed command height in meters.")
    parser.add_argument(
        "--takeoff_duration",
        type=float,
        default=3.0,
        help="Takeoff duration in seconds. Use <=0 to skip takeoff.",
    )
    parser.add_argument("--rate_hz", type=float, default=20.0, help="cmd_position publish rate.")
    parser.add_argument(
        "--reference_timeout",
        type=float,
        default=0.5,
        help="Warn if /cf<ID>/reference is older than this many seconds.",
    )
    parser.add_argument(
        "--state_timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for /cf<ID>/state before takeoff. Use <=0 to bypass.",
    )
    parser.add_argument(
        "--done_topic",
        default="/seamlis/exploration_done",
        help="Bool topic that triggers landing when true.",
    )
    land_group = parser.add_mutually_exclusive_group()
    land_group.add_argument("--land_on_done", dest="land_on_done", action="store_true", help="Land when done_topic is true.")
    land_group.add_argument("--no_land_on_done", dest="land_on_done", action="store_false", help="Do not land on done_topic.")
    parser.set_defaults(land_on_done=True)
    parser.add_argument("--landing_height", type=float, default=0.04, help="Landing target height in meters.")
    parser.add_argument("--landing_duration", type=float, default=3.0, help="Landing duration in seconds.")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("cf_takeoff_reference_bridge", anonymous=True)

    cf_ids = parse_cf_ids(args.cf_ids) or get_cf_ids()
    bridge = FixedHeightReferenceBridge(
        cf_ids=cf_ids,
        height=args.height,
        takeoff_duration=args.takeoff_duration,
        rate_hz=args.rate_hz,
        reference_timeout=args.reference_timeout,
        state_timeout=args.state_timeout,
        done_topic=args.done_topic,
        land_on_done=args.land_on_done,
        landing_height=args.landing_height,
        landing_duration=args.landing_duration,
    )
    bridge.takeoff_all()
    bridge.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
