/*
 * File:          hardware.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef CONTROLLERS_YOUBOT_HARDWARE_H_
#define CONTROLLERS_YOUBOT_HARDWARE_H_

/*
 * controller includes
 */
#include "config.h"

/*
 * webots includes
 */
#include <webots/Robot.hpp>
#include <webots/Servo.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

/*
 * c++ includes
 */
#include <vector>
#include <string>

/*
 * boost includes
 */
#include <boost/units/systems/si.hpp>
#include <boost/units/systems/si/prefixes.hpp>

/*
 * youbot specific includes
 */
#include <base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp>
#include <base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp>

class Hardware {
    public:
        struct device{
                std::string name;
                std::string topicName;
                bool sendTransforms;
                std::string transformReference;
                double rotation[3];
                double translation[3];
        };

        Hardware(Config* conf);
        std::vector<webots::Servo*> wheel;
        std::vector<webots::Servo*> armJoint;
        std::vector<webots::Servo*> handGripper;
        std::vector<webots::Camera*> laserScanner;
        std::vector<webots::Camera*> camera;
        std::vector<webots::Camera*> kinect;
        std::vector<webots::GPS*> gps;
        std::vector<webots::Compass*> compass;
        
        std::vector<boost::units::quantity<boost::units::si::angular_velocity> > globalWheelVelocities;
        youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;
        
    private:
        youbot::FourSwedishWheelOmniBaseKinematicConfiguration youBotBaseKinematicConfig;
};
#endif  // CONTROLLERS_YOUBOT_HARDWARE_H_
