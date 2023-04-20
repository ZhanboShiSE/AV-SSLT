#!usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import message_filters
from DoA_msgs.msg import Source
from audiovisual_integrated_robot.msg import AVSSLT_Awake


def msgs_frequency_control_callback(msg1, msg2):
    pub = rospy.Publisher('AVSSLT_HarkSource', Source, queue_size=10)
    pub.publish(msg1)


def main():
    rospy.init_node('DoAMsgsFrequencyControlNode', anonymous=True)
    source_sub = message_filters.Subscriber('Source', Source)
    clock_sub = message_filters.Subscriber('AVSSLT_Clock', AVSSLT_Awake)
    ts = message_filters.ApproximateTimeSynchronizer([source_sub, clock_sub], 10, 0.05)
    ts.registerCallback(msgs_frequency_control_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass