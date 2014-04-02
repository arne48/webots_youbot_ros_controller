/*
 * File:          config.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef CONTROLLERS_YOUBOT_CONFIG_H_
#define CONTROLLERS_YOUBOT_CONFIG_H_

/*
 * boost includes
 */
#include <boost/units/systems/si.hpp>
#include <boost/units/systems/si/prefixes.hpp>

/*
 * c++ includes
 */
#include <string>
#include <vector>

class Config {
    public:
        struct device {
            std::string name;
            std::string topicName;
            bool sendTransforms;
            std::string transformReference;
            double rotation[3];
            double translation[3];
        };

        Config(std::string path);
        int timeStep;
        size_t baseJoints;
        double pi;
        double oneDegRad;
        double laserTimeIncrement;
        double laserScanTime;
        double laserMinRange;
        double laserMaxRange;
        double armJoint0Offset;
        double armJoint1Offset;
        double armJoint2Offset;
        double armJoint3Offset;
        double armJoint4Offset;
        double baseRotationRatio;
        double baseSlideRatio;
        double angle_max_map;
        double angle_min_map;
        boost::units::quantity<boost::units::si::length> baseDistFrontRearWheels;
        boost::units::quantity<boost::units::si::length> baseDistFrontWheels;
        boost::units::quantity<boost::units::si::length> baseWheelRadius;
        std::vector<std::string> wheelNames;
        std::vector<std::string> armNames;
        std::vector<std::string> fingerNames;
        std::vector<std::string> cameraNames;
        std::vector<std::string> kinectNames;
        std::vector<std::string> laserScannerNames;
        std::string odometryTopicName;
        std::vector<std::vector<device> > devDetails;

    private:
};
#endif  // CONTROLLERS_YOUBOT_CONFIG_H_


