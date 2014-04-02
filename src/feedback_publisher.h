/*
 * File:          feedback_publisher.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef CONTROLLERS_YOUBOT_FEEDBACK_PUBLISHER_H_
#define CONTROLLERS_YOUBOT_FEEDBACK_PUBLISHER_H_

/*
 * ros includes
 */
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"
#include "sensor_laserscanner.h"
#include "sensor_kinect.h"
#include "sensor_camera.h"


/*
 * c++ includes
 */
#include <vector>

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

class FeedbackPublisher {
    public:
        FeedbackPublisher(Config* conf, Hardware* hardware, ros::NodeHandle nh,
                                        std::vector<Sensor_Kinect*>* kinect,
                                        std::vector<Sensor_Camera*>* camera,
                                        std::vector<Sensor_Laserscanner*>* laserScanner);
        void run();
        void initiatePositions();
    
	private:
		std::vector<double> origin_position;
		std::vector<double> origin_rotation;
		double getRelativeRotation(double xaxis, double yaxis, double origin_xaxis, double origin_yaxis);
        double pi;
        double deg2rad(double deg);
        float armPosFromWebots(int arm);
        float armPosFromWebots(int arm, float webotsArmPos);
        sensor_msgs::JointState joint_state;
        tf::TransformBroadcaster tfbroad;
        ros::Publisher odometryPublisher;
        ros::Publisher pub_joint_states;
        std::vector<boost::units::quantity<boost::units::si::plane_angle> > wheel_positions;
        Config* conf;
        Hardware* hardware;
        std::vector<Sensor_Kinect*>* kinect;
        std::vector<Sensor_Camera*>* camera;
        std::vector<Sensor_Laserscanner*>* laserScanner;
};
#endif  // CONTROLLERS_YOUBOT_FEEDBACK_PUBLISHER_H_

