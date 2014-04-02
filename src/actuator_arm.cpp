/*
 * File:          actuator_arm.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"
#include "actuator_arm.h"

/*
 * ros includes
 */
#include "ros/ros.h"
#include <brics_actuator/JointPositions.h>

Actuator_Arm::Actuator_Arm(Config* conf, Hardware* hardware, ros::NodeHandle nh) {
    this->hardware = hardware;
    this->conf= conf;
}
void Actuator_Arm::armCommandCallback(const brics_actuator::JointPositions& youBotJointPositions) {
    int size = youBotJointPositions.positions.size();
    for (int i = 0; i < size; i++) {
        hardware->armJoint[i]->setPosition(armPosToWebots(i, youBotJointPositions.positions[i].value));
    }
}
void Actuator_Arm::gripperCommandCallback(const brics_actuator::JointPositions& youBotJointPositions) {
    int size = youBotJointPositions.positions.size();
    for (int i = 0; i < size; i++) {
        hardware->handGripper[i]->setPosition(youBotJointPositions.positions[i].value);
    }
}

float Actuator_Arm::armPosToWebots(int arm, float armPos) {
    switch (arm) {
        case(0):
            armPos += conf->armJoint0Offset;
            break;
        case(1):
            armPos += conf->armJoint1Offset;
            break;
        case(2):
            armPos += conf->armJoint2Offset;
            break;
        case(3):
            armPos += conf->armJoint3Offset;
            break;
        case(4):
            armPos += conf->armJoint4Offset;
            break;
        default:
            break;
    }
    armPos *= -1;
    return armPos;
}
