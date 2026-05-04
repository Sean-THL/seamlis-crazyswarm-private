#!/usr/bin/env python3
import rospy
from crazyswarm.msg import Position

from cf_state_subscriber import get_cf_ids


class CfReferencePublisher:
    def __init__(self, cf_ids=None):
        self.cf_ids = list(cf_ids) if cf_ids is not None else get_cf_ids()
        if not self.cf_ids:
            raise RuntimeError("No Crazyflie IDs found in ROS param 'crazyflies'. Launch crazyswarm first.")

        self.cmd_publishers = {
            cf_id: rospy.Publisher(f"/cf{cf_id}/cmd_position", Position, queue_size=10)
            for cf_id in self.cf_ids
        }
        self.subscribers = []

        for cf_id in self.cf_ids:
            sub = rospy.Subscriber(
                f"/cf{cf_id}/reference",
                Position,
                self._make_reference_callback(cf_id),
                queue_size=1,
            )
            self.subscribers.append(sub)

        rospy.loginfo(
            "CfReferencePublisher forwarding: %s",
            ", ".join(f"/cf{cf_id}/reference -> /cf{cf_id}/cmd_position" for cf_id in self.cf_ids),
        )

    def _make_reference_callback(self, cf_id):
        def callback(msg):
            cmd = Position()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = msg.header.frame_id or "world"
            cmd.x = float(msg.x)
            cmd.y = float(msg.y)
            cmd.z = float(msg.z)
            cmd.yaw = float(msg.yaw)
            self.cmd_publishers[cf_id].publish(cmd)

        return callback

    def publish_reference(self, cf_id, position, yaw):
        msg = Position()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.x = float(position[0])
        msg.y = float(position[1])
        msg.z = float(position[2])
        msg.yaw = float(yaw)
        self.cmd_publishers[cf_id].publish(msg)


def main():
    rospy.init_node("cf_reference_publish", anonymous=True)
    CfReferencePublisher()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
