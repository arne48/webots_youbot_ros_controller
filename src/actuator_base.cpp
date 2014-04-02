/*
 * File:          actuator_base.cpp
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
#include "actuator_base.h"

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
 * youbot specific includes
 */
#include <base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp>
#include <base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp>

/*
 * c++ includes
 */
#include <limits>
#include <vector>

/*
 * namespaces
 */ 
using namespace boost::units;
using namespace boost::units::si;
using namespace boost::units::angle;
using boost::units::si::meters;
using namespace webots;
using namespace std;

Actuator_Base::Actuator_Base(Config* conf, Hardware* hardware, ros::NodeHandle nh) {
    this->hardware = hardware;
    this->conf = conf;
}

void Actuator_Base::baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand) {
    quantity<si::velocity> longitudinalVelocity    = youbotBaseCommand.linear.x * meter_per_second;
    quantity<si::velocity> transversalVelocity     = youbotBaseCommand.linear.y * meter_per_second;
    quantity<si::angular_velocity> angularVelocity = youbotBaseCommand.angular.z * radian_per_second;
    setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}

void Actuator_Base::setBaseVelocity(const quantity<si::velocity>& longitudinalVelocity, const quantity<si::velocity>& transversalVelocity,
                                    const quantity<si::angular_velocity>& angularVelocity) {
    vector<quantity<angular_velocity> > wheelVelocities;

    hardware->youBotBaseKinematic.cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity,
                                                                     angularVelocity, wheelVelocities);

    if (wheelVelocities.size() < conf->baseJoints)
        throw std::out_of_range("To less wheel velocities");

    // Enable velocity control mode
    hardware->wheel[0]->setPosition(std::numeric_limits<double>::infinity());
    hardware->wheel[1]->setPosition(std::numeric_limits<double>::infinity());
    hardware->wheel[2]->setPosition(std::numeric_limits<double>::infinity());
    hardware->wheel[3]->setPosition(std::numeric_limits<double>::infinity());

    for (int i = 0; i < 4; i++) {
        if (i % 2 == 0) {
            hardware->globalWheelVelocities[i] = -wheelVelocities[i];
        } else {
            hardware->globalWheelVelocities[i] = wheelVelocities[i];
        }
    }

    // Set wheel velocities
    hardware->wheel[0]->setVelocity(wheelVelocities[1].value());
    hardware->wheel[1]->setVelocity(-wheelVelocities[0].value());
    hardware->wheel[2]->setVelocity(wheelVelocities[3].value());
    hardware->wheel[3]->setVelocity(-wheelVelocities[2].value());
}
