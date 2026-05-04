#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np
import rospy
from crazyswarm.msg import Position


@dataclass
class RobotState:
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    stamp: rospy.Time


def angle_normalize(angle):
    return ((angle + math.pi) % (2.0 * math.pi)) - math.pi


def get_cf_ids():
    crazyflies = rospy.get_param("crazyflies", [])
    ids = []
    for entry in crazyflies:
        if isinstance(entry, dict) and "id" in entry:
            ids.append(int(entry["id"]))
    return ids


class CfStateTracker:
    def __init__(self, cf_ids=None, topic_suffix="/state", velocity_alpha=0.5, queue_size=1):
        self.cf_ids = list(cf_ids) if cf_ids is not None else get_cf_ids()
        if not self.cf_ids:
            raise RuntimeError("No Crazyflie IDs found in ROS param 'crazyflies'. Launch crazyswarm first.")

        self.topic_suffix = str(topic_suffix)
        self.velocity_alpha = float(velocity_alpha)
        self.latest_states = {cf_id: None for cf_id in self.cf_ids}
        self.prev_measurements = {cf_id: None for cf_id in self.cf_ids}
        self.state_received = {cf_id: False for cf_id in self.cf_ids}
        self.subscribers = []

        for cf_id in self.cf_ids:
            topic_name = f"/cf{cf_id}{self.topic_suffix}"
            sub = rospy.Subscriber(topic_name, Position, self._make_callback(cf_id), queue_size=queue_size)
            self.subscribers.append(sub)

        rospy.loginfo(
            "CfStateTracker subscribed to: %s",
            ", ".join(f"/cf{cf_id}{self.topic_suffix}" for cf_id in self.cf_ids),
        )

    def _make_callback(self, cf_id):
        def callback(msg):
            stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            measurement = np.array([float(msg.x), float(msg.y)], dtype=float)

            vx = 0.0
            vy = 0.0
            prev = self.prev_measurements[cf_id]
            if prev is not None:
                prev_xy, prev_stamp, prev_vel = prev
                dt = (stamp - prev_stamp).to_sec()
                if dt > 1e-4:
                    raw_vel = (measurement - prev_xy) / dt
                    vel = raw_vel if prev_vel is None else (
                        self.velocity_alpha * raw_vel + (1.0 - self.velocity_alpha) * prev_vel
                    )
                    vx = float(vel[0])
                    vy = float(vel[1])
                elif prev_vel is not None:
                    vx = float(prev_vel[0])
                    vy = float(prev_vel[1])

            vel_vec = np.array([vx, vy], dtype=float)
            self.prev_measurements[cf_id] = (measurement, stamp, vel_vec)
            self.latest_states[cf_id] = RobotState(
                x=float(msg.x),
                y=float(msg.y),
                z=float(msg.z),
                yaw=angle_normalize(float(msg.yaw)),
                vx=vx,
                vy=vy,
                stamp=stamp,
            )
            self.state_received[cf_id] = True

        return callback

    def all_states_ready(self):
        return all(self.state_received.values())

    def get_state(self, cf_id):
        return self.latest_states.get(cf_id)

    def get_all_states(self):
        return dict(self.latest_states)


def main():
    rospy.init_node("cf_state_subscriber", anonymous=True)
    velocity_alpha = float(rospy.get_param("~velocity_alpha", 0.5))
    tracker = CfStateTracker(velocity_alpha=velocity_alpha)
    rate_hz = float(rospy.get_param("~log_rate_hz", 2.0))
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        if tracker.all_states_ready():
            summary = []
            for cf_id in tracker.cf_ids:
                state = tracker.get_state(cf_id)
                summary.append(
                    f"cf{cf_id}: x={state.x:.3f} y={state.y:.3f} z={state.z:.3f} "
                    f"yaw={state.yaw:.3f} vx={state.vx:.3f} vy={state.vy:.3f}"
                )
            rospy.loginfo_throttle(1.0, " | ".join(summary))
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
