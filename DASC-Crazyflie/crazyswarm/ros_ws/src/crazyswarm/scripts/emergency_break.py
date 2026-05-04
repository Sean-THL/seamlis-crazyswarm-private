#!/usr/bin/env python


'''Constantly sends the status of the system as True (safe), until the KeyboardInterrupt, 
which then sends False (unsafe) and terminates'''

import rospy
from std_msgs.msg import Bool

pub = None 


# Publishes the status as False to Crazyswarm node
def on_shutdown():
    rospy.loginfo("Shutting down. Publishing swarm status: False")
    if pub is not None:
        for _ in range(100):
            pub.publish(Bool(False))
            rospy.sleep(0.1)


# Publishes the status as True to Crazyswarm node until there is a keyboard interrupt.
def main():
    global pub
    rospy.init_node('status_publisher', anonymous=True)
    pub = rospy.Publisher("/swarm/status", Bool, queue_size=1)
    rospy.on_shutdown(on_shutdown)
    
    rate = rospy.Rate(100)  # 100 Hz
    rospy.loginfo("Publishing swarm status: True (active)")
    
    while not rospy.is_shutdown():
        pub.publish(Bool(True))
        rate.sleep()

if __name__ == "__main__":
    main()
