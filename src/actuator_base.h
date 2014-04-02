/*
 * File:          actuator_base.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de

#ifndef CONTROLLERS_YOUBOT_ACTUATOR_BASE_H_
#define CONTROLLERS_YOUBOT_ACTUATOR_BASE_H_

/*
 * ros includes
 */
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

/*
 * boost includes
 */
#include <boost/units/systems/si.hpp>
#include <boost/units/systems/si/prefixes.hpp>

/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"

class Actuator_Base {
    public:
        Actuator_Base(Config* conf, Hardware* hardware, ros::NodeHandle nh);
        void baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand);
        
    private:
        void setBaseVelocity(const boost::units::quantity<boost::units::si::velocity>& longitudinalVelocity,
                                const boost::units::quantity<boost::units::si::velocity>& transversalVelocity,
                                const boost::units::quantity<boost::units::si::angular_velocity>& angularVelocity);
        Hardware* hardware;
        Config* conf;
};
#endif  // CONTROLLERS_YOUBOT_ACTUATOR_BASE_H_
