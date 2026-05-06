#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import rospy
from crazyswarm.msg import Position


DEFAULT_FLIGHT_Z = 1.0


def _parse_cf_ids(raw_ids):
    return [int(item.strip()) for item in raw_ids.split(",") if item.strip()]


def _angle_normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay a SEAMLiS simulation trajectory CSV as Crazyswarm /cf<ID>/cmd_position commands."
    )
    parser.add_argument("--trajectory", required=True, help="CSV created by seamlis/record_sim_trajectory.py.")
    parser.add_argument("--cf_ids", default=None, help="Comma-separated CF IDs to replay. Defaults to all IDs in CSV.")
    parser.add_argument("--z", type=float, default=DEFAULT_FLIGHT_Z, help="Fixed flight height for all commands.")
    parser.add_argument("--rate_hz", type=float, default=30.0, help="Command publish rate.")
    parser.add_argument("--time_scale", type=float, default=1.0, help=">1 slower replay, <1 faster replay.")
    parser.add_argument("--settle_time", type=float, default=2.0, help="Seconds to hold the first waypoint before replay.")
    parser.add_argument("--hold_final_time", type=float, default=2.0, help="Seconds to hold final waypoint after replay.")
    parser.add_argument("--publish_yaw", action="store_true", help="Use yaw recorded in the CSV. Default holds yaw=0.")
    parser.add_argument("--dry_run", action="store_true", help="Print commands without publishing them.")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def _load_trajectory(path):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    trajectories = defaultdict(list)
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"time", "cf_id", "vicon_x", "vicon_y", "yaw"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Trajectory CSV is missing columns: {sorted(missing)}")

        for row in reader:
            cf_id = int(row["cf_id"])
            trajectories[cf_id].append(
                {
                    "time": float(row["time"]),
                    "x": float(row["vicon_x"]),
                    "y": float(row["vicon_y"]),
                    "yaw": float(row["yaw"]) if row.get("yaw") not in (None, "") else 0.0,
                }
            )

    for cf_id, points in trajectories.items():
        points.sort(key=lambda item: item["time"])
        if len(points) == 0:
            raise ValueError(f"No trajectory points for cf{cf_id}")
    return dict(trajectories)


def _sample(points, t):
    if t <= points[0]["time"]:
        return points[0]
    if t >= points[-1]["time"]:
        return points[-1]

    lo = 0
    hi = len(points) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if points[mid]["time"] <= t:
            lo = mid
        else:
            hi = mid

    p0 = points[lo]
    p1 = points[hi]
    dt = max(p1["time"] - p0["time"], 1e-9)
    alpha = (t - p0["time"]) / dt
    yaw_delta = _angle_normalize(p1["yaw"] - p0["yaw"])
    return {
        "time": t,
        "x": p0["x"] + alpha * (p1["x"] - p0["x"]),
        "y": p0["y"] + alpha * (p1["y"] - p0["y"]),
        "yaw": _angle_normalize(p0["yaw"] + alpha * yaw_delta),
    }


def _publish_point(publisher, cf_id, point, z, publish_yaw, dry_run):
    yaw = point["yaw"] if publish_yaw else 0.0
    if dry_run:
        rospy.loginfo("dry_run cf%s cmd: x=%.3f y=%.3f z=%.3f yaw=%.3f", cf_id, point["x"], point["y"], z, yaw)
        return

    msg = Position()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.x = float(point["x"])
    msg.y = float(point["y"])
    msg.z = float(z)
    msg.yaw = float(yaw)
    publisher.publish(msg)


def main():
    rospy.init_node("follow_sim_trajectory", anonymous=True)
    args = parse_args()
    trajectories = _load_trajectory(args.trajectory)

    if args.cf_ids is None:
        cf_ids = sorted(trajectories.keys())
    else:
        cf_ids = _parse_cf_ids(args.cf_ids)

    missing = [cf_id for cf_id in cf_ids if cf_id not in trajectories]
    if missing:
        raise ValueError(f"Requested CF IDs not found in trajectory CSV: {missing}")

    publishers = {
        cf_id: rospy.Publisher(f"/cf{cf_id}/cmd_position", Position, queue_size=10)
        for cf_id in cf_ids
    }

    duration = max(trajectories[cf_id][-1]["time"] for cf_id in cf_ids)
    rate = rospy.Rate(args.rate_hz)
    rospy.loginfo(
        "Replaying trajectory for cf_ids=%s duration=%.2fs z=%.2f time_scale=%.2f dry_run=%s",
        cf_ids,
        duration,
        args.z,
        args.time_scale,
        args.dry_run,
    )

    settle_end = rospy.Time.now() + rospy.Duration(max(args.settle_time, 0.0))
    while not rospy.is_shutdown() and rospy.Time.now() < settle_end:
        for cf_id in cf_ids:
            _publish_point(publishers[cf_id], cf_id, trajectories[cf_id][0], args.z, args.publish_yaw, args.dry_run)
        rate.sleep()

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec() / max(args.time_scale, 1e-6)
        if elapsed > duration:
            break
        for cf_id in cf_ids:
            point = _sample(trajectories[cf_id], elapsed)
            _publish_point(publishers[cf_id], cf_id, point, args.z, args.publish_yaw, args.dry_run)
        rospy.loginfo_throttle(1.0, "Trajectory replay active: t=%.2f / %.2f", elapsed, duration)
        rate.sleep()

    hold_end = rospy.Time.now() + rospy.Duration(max(args.hold_final_time, 0.0))
    while not rospy.is_shutdown() and rospy.Time.now() < hold_end:
        for cf_id in cf_ids:
            _publish_point(publishers[cf_id], cf_id, trajectories[cf_id][-1], args.z, args.publish_yaw, args.dry_run)
        rate.sleep()

    rospy.loginfo("Trajectory replay finished.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
