/*
 * File:          youbot.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef  CONTROLLERS_YOUBOT_YOUBOT_H_
#define  CONTROLLERS_YOUBOT_YOUBOT_H_
/*
 * webots includes
 */
#include <webots/Robot.hpp>

/*
 * controller includes
 */
#include "sensor_laserscanner.h"
#include "sensor_kinect.h"
#include "sensor_camera.h"
#include "actuator_base.h"
#include "actuator_arm.h"
#include "feedback_publisher.h"
#include "hardware.h"
#include "config.h"

/*
 * ros includes
 */
#include "ros/ros.h"

/*
 * c++ includes
 */
#include <vector>


class YouBot : public webots::Robot {
    public:
        YouBot(ros::NodeHandle nh);
        virtual ~YouBot();
        void run();
        void initiatePositions();
 
   private:
        ros::NodeHandle nh;
        std::vector<Sensor_Kinect*> kinect;
        std::vector<Sensor_Camera*> camera;
        std::vector<Sensor_Laserscanner*> laserScanner;
        Hardware* hardware;
        Config* conf;
        FeedbackPublisher* feedback;
        Actuator_Base* base;
        Actuator_Arm* arm;
        ros::Subscriber sub_basecmd;
        ros::Subscriber sub_armcmd;
        ros::Subscriber sub_grippercmd;
};
#endif  // CONTROLLERS_YOUBOT_YOUBOT_H_
