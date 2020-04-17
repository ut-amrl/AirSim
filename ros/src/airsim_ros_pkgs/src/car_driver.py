#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/airsim_node/PhysXCar/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    t = Twist()
    t.linear.x = 5
    t.linear.y = 0
    t.linear.z = 0

    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0

    pub.publish(Twist())
    rate = rospy.Rate(.0005)
    while not rospy.is_shutdown():
        rospy.loginfo(t)
        pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass