/*
 * File:          actuator_arm.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 

#ifndef CONTROLLERS_YOUBOT_ACTUATOR_ARM_H_
#define CONTROLLERS_YOUBOT_ACTUATOR_ARM_H_

/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"

/*
 * ros includes
 */
#include "ros/ros.h"
#include <brics_actuator/JointPositions.h>

class Actuator_Arm {
    public:
        Actuator_Arm(Config* conf, Hardware* hardware, ros::NodeHandle nh);
        void armCommandCallback(const brics_actuator::JointPositions& youBotJointPositions);
        void gripperCommandCallback(const brics_actuator::JointPositions& youBotJointPositions);
    private:
        float armPosToWebots(int arm, float armPos);
        Hardware* hardware;
        Config* conf;
};
#endif  // CONTROLLERS_YOUBOT_ACTUATOR_ARM_H_

