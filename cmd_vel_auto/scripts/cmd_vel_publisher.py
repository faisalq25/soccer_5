#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def publish_cmd_vel():
    rospy.init_node('auto_cmd_vel_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    forward_twist = Twist()
    forward_twist.linear.x = 2.0

    backward_twist = Twist()
    backward_twist.angular.z = -2.0

    while not rospy.is_shutdown():
        # Maju 3 detik
        start_time = time.time()
        while time.time() - start_time < 5:
            pub.publish(forward_twist)
            rospy.loginfo("Maju")
            rate.sleep()

        # Mundur 3 detik
        start_time = time.time()
        while time.time() - start_time < 2:
            pub.publish(backward_twist)
            rospy.loginfo("Mundur")
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_cmd_vel()
    except rospy.ROSInterruptException:
        pass

