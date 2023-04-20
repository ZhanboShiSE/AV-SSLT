#include <iostream>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <chrono>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "cv_bridge/cv_bridge.h"
#include "audiovisual_integrated_robot/AVIR_Awake.h"

#include "image_ros_read.hpp"
#include "facetracing.hpp"

using namespace std;
using namespace cv;

void imgread_msg_callback(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat img = cv_ptr->image;

    if (img.empty())
        cout << "Ros Message Image Empyt" << endl;
    
    ftFindFace(img, "shizhanbo");
    
    imshow("RGB", img);

    if ((waitKey(10) & 0xFF) == 27)
        ros::shutdown();
}

void depthread_msg_callback(const sensor_msgs::Image::ConstPtr &msg, int argc, char ** argv, int &step) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat img = cv_ptr->image;

    if (img.empty())
        cout << "Ros Message Image Empyt" << endl;
    
    ftCalculateDepth(img, argc, argv, step);
    // imshow("Depth", img);

    if ((waitKey(10) & 0xFF) == 27)
        ros::shutdown();
}

void awake_msg_callback(const audiovisual_integrated_robot::AVIR_Awake::ConstPtr &msg) {
    if (msg->is_awake == 1) {
        ros::Duration(2.0).sleep();
        ftSetAwake(1);
        ftSetFinish(0);
    }
        
}

void msgRosSubscribe(int argc, char ** argv) {
    ros::init(argc, argv, "CameraSubscribeNode", ros::init_options::AnonymousName);
    ros::NodeHandle rgb_handle, depth_handle, awake_handle;
    ros::Subscriber rgb_sub = rgb_handle.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 30, imgread_msg_callback);
    ros::CallbackQueue queue_depth, queue_awake;
    depth_handle.setCallbackQueue(&queue_depth);
    awake_handle.setCallbackQueue(&queue_awake);
    int step = 0;
    ros::Subscriber depth_sub = depth_handle.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 10, boost::bind(depthread_msg_callback, _1, argc, argv, step));
    ros::Subscriber awake_sub = awake_handle.subscribe<audiovisual_integrated_robot::AVIR_Awake>("Awake", 1, awake_msg_callback);

    thread spinner_thread_depth([&queue_depth]() {
        ros::SingleThreadedSpinner spinner_a;
        spinner_a.spin(&queue_depth);
    });
    thread spinner_thread_awake([&queue_awake]() {
        ros::SingleThreadedSpinner spinner_b;
        spinner_b.spin(&queue_awake);
    });
    ros::spin();
    spinner_thread_awake.join();
    spinner_thread_depth.join();
}

void msgMoveRosPublish(int argc, char ** argv, int rotation) {
    ros::init(argc, argv, "MovePublishNode", ros::init_options::AnonymousName);
    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel", 10, true);
    
    geometry_msgs::Twist move_cmd  = geometry_msgs::Twist();
    // because the camera's orientation is opposite to the robot's orientation
    // add -0.3 for linear x to move forword for camera's coordinate system
    // the same as the angular velocity
    move_cmd.linear.x = -0.3;

    if (rotation != 0) 
        move_cmd.angular.z = -rotation * 0.2;

    pub.publish(move_cmd);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}

void msgMoveForward(int argc, char ** argv) {
    ros::init(argc, argv, "MoveForwardNode", ros::init_options::AnonymousName);
    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel", 10, true);
    
    geometry_msgs::Twist move_cmd  = geometry_msgs::Twist();
    move_cmd.linear.x = -0.3;

    pub.publish(move_cmd);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}

void msgMoveLeft(int argc, char ** argv) {
    ros::init(argc, argv, "MoveLeftNode", ros::init_options::AnonymousName);
    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel", 10, true);
    
    geometry_msgs::Twist move_cmd  = geometry_msgs::Twist();
    move_cmd.linear.x = -0.2;
    move_cmd.angular.z = 0.3;

    pub.publish(move_cmd);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}

void msgMoveRight(int argc, char ** argv) {
    ros::init(argc, argv, "MoveLeftNode", ros::init_options::AnonymousName);
    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel", 10, true);
    
    geometry_msgs::Twist move_cmd  = geometry_msgs::Twist();
    move_cmd.linear.x = -0.2;
    move_cmd.angular.z = -0.3;

    pub.publish(move_cmd);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}
