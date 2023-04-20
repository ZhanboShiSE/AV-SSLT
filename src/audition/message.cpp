#include <iostream>
#include <cstdio>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Header.h"

#include "audition/message.hpp"
#include "audiovisual_integrated_robot/AVIR_Awake.h"

using std::cout;
using std::endl;

const int MAX_NAME_LEN      = 256;

auto start = std::chrono::system_clock::now();
auto end = std::chrono::system_clock::now();
auto elapsed1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


void msgAwakeRosPublish(int argc, char ** argv) {
    ros::init(argc, argv, "AwakePublishNode", ros::init_options::AnonymousName);
    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<audiovisual_integrated_robot::AVIR_Awake>("Awake", 10, true);

    audiovisual_integrated_robot::AVIR_Awake is_awake = audiovisual_integrated_robot::AVIR_Awake();
    
    ros::Duration(1.0).sleep();

    is_awake.is_awake = 1;
    is_awake.header.stamp = ros::Time::now() - ros::Duration(1.0);

    pub.publish(is_awake);
    ros::spinOnce();
    ros::shutdown();
}