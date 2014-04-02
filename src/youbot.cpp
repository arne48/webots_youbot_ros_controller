/*
 * File:          youbot.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */


/*
 * ros includes
 */
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

/*
 * c++ includes
 */
#include <string>
#include <vector>

/*
 * controller includes
 */
#include "youbot.h"
#include "config.h"
#include "hardware.h"
#include "feedback_publisher.h"
#include "sensor_laserscanner.h"
#include "sensor_camera.h"
#include "sensor_kinect.h"
#include "actuator_base.h"
#include "actuator_arm.h"


#include <webots/GPS.hpp>

/*
 * namespaces
 */ 
using namespace webots;
using namespace std;

YouBot::YouBot(ros::NodeHandle nh) {
    this->nh = nh;
    /*
     * Robot Init
     */
    conf = new Config(string("config.xml"));
    hardware = new Hardware(conf);

    /*
     * ROS Publishers
     */
    for (size_t i = 0; i < hardware->laserScanner.size(); i++) {
        laserScanner.push_back(new Sensor_Laserscanner(conf, hardware, nh, i));
    }

    for (size_t i = 0; i < hardware->camera.size(); i++) {
        camera.push_back(new Sensor_Camera(conf, hardware, nh, i));
    }

    for (size_t i = 0; i < hardware->kinect.size(); i++) {
        kinect.push_back(new Sensor_Kinect(conf, hardware, nh, i));
    }

    feedback = new FeedbackPublisher(conf, hardware, nh, &kinect, &camera, &laserScanner);
    
    /*
     * ROS Subscribers
     */	
    base = new Actuator_Base(conf, hardware, nh);
    arm = new Actuator_Arm(conf, hardware, nh);
    
    sub_basecmd = nh.subscribe("cmd_vel", 1000, &Actuator_Base::baseCommandCallback, base);
    sub_armcmd = nh.subscribe("/arm_1/arm_controller/position_command", 1000, &Actuator_Arm::armCommandCallback, arm);
    sub_grippercmd = nh.subscribe("/arm_1/gripper_controller/position_command", 1000,  &Actuator_Arm::gripperCommandCallback, arm);
}

YouBot::~YouBot() {
	delete conf;
    delete hardware;
    delete feedback;
    delete base;
    delete arm;
}

void YouBot::run() {
    feedback->run();

    for (size_t i = 0; i < laserScanner.size(); i++) {
		laserScanner[i]->run();
    }

    for (size_t i = 0; i < camera.size(); i++) {
		camera[i]->run();
    }

    for (size_t i = 0; i < kinect.size(); i++) {
		kinect[i]->run();
    }

    ros::spinOnce();
}

void YouBot::initiatePositions() {
	feedback->initiatePositions();
}
