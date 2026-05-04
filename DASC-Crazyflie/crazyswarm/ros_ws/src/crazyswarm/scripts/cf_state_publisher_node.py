#!/usr/bin/env python3
import math

import rospy
import tf
from crazyswarm.msg import Position


def get_cf_ids():
    crazyflies = rospy.get_param("crazyflies", [])
    ids = []
    for entry in crazyflies:
        if isinstance(entry, dict) and "id" in entry:
            ids.append(int(entry["id"]))
    return ids


def main():
    rospy.init_node("cf_state_publisher", anonymous=True)

    cf_ids = get_cf_ids()
    if not cf_ids:
        rospy.logerr("No Crazyflie IDs found in ROS param 'crazyflies'. Launch crazyswarm first.")
        return

    listener = tf.TransformListener()
    publishers = {
        cf_id: rospy.Publisher(f"/cf{cf_id}/state", Position, queue_size=10)
        for cf_id in cf_ids
    }

    rospy.loginfo("Publishing TF-backed state topics for Crazyflie IDs: %s", ", ".join(map(str, cf_ids)))
    rospy.loginfo("Echo positions with: rostopic echo /cf<ID>/state")

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for cf_id in cf_ids:
            try:
                position, quaternion = listener.lookupTransform("/world", f"/cf{cf_id}", rospy.Time(0))
                _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

                state = Position()
                state.header.stamp = now
                state.header.frame_id = "world"
                state.x = float(position[0])
                state.y = float(position[1])
                state.z = float(position[2])
                state.yaw = yaw if math.isfinite(yaw) else 0.0
                print(f"Publishing state for cf{cf_id}: x={state.x:.2f}, y={state.y:.2f}, z={state.z:.2f}, yaw={math.degrees(state.yaw):.1f}°")
                publishers[cf_id].publish(state)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "Waiting for TF /world -> /cf%s for VICON-backed state publishing: %s",
                    cf_id,
                    exc,
                )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
