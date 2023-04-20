#!usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from audiovisual_integrated_robot.msg import AVSSLT_Awake


def main():
    rospy.init_node('AVSSLTClockNode', anonymous=True)
    pub = rospy.Publisher('AVSSLT_Clock', AVSSLT_Awake, queue_size=10)
    rate = rospy.Rate(50) # 10Hz
    while not rospy.is_shutdown():
        clock = AVSSLT_Awake()
        clock.is_awake = 1
        now = rospy.get_rostime()
        clock.header.stamp.secs = now.secs
        clock.header.stamp.nsecs = now.nsecs
        pub.publish(clock)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass