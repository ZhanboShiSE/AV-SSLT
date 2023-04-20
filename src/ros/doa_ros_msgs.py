#!usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import message_filters
from DoA_msgs.msg import Source
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from audiovisual_integrated_robot.msg import AVSSLT_Awake
# from subprocess import call


def msgs_filter_callback(msg1, msg2):
    for i in msg1.src:
        id_, power, x, y, z, = i.id, i.power, i.x, i.y, i.z
        azimuth, elevation = i.azimuth, i.elevation
        
        flag = 1
        if azimuth < 0:
            flag = -1
        
        azimuth = abs(azimuth)
        count = int(azimuth / 10)
        
        for j in range(count):
            azimuth_publish(flag*1.5)
            rospy.sleep(0.1)
        
        print("azimuth is"+ str(azimuth))
        
        break


def azimuth_publish(ang_speed):
    cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=10, latch=True)
    move_cmd = Twist()
    move_cmd.angular.z = ang_speed
    cmd_vel.publish(move_cmd)


def main():
    rospy.init_node('AwakeSSLFilterNode', anonymous=True)
    source_sub = message_filters.Subscriber('AVSSLT_Source', Source)
    awake_sub = message_filters.Subscriber('Awake', AVSSLT_Awake)
    ts = message_filters.ApproximateTimeSynchronizer([source_sub, awake_sub], 50, 0.1)
    ts.registerCallback(msgs_filter_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass