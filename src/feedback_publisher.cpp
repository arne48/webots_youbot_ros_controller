/*
 * File:          feedback_publisher.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

/*
 * webots includes
 */
#include <webots/Robot.hpp>

/*
 * ros includes
 */
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

/*
 * controller includes
 */
#include "feedback_publisher.h"
#include "config.h"
#include "hardware.h"

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
 * namespaces
 */ 
using namespace boost::units;
using namespace boost::units::si;
using namespace boost::units::angle;
using boost::units::si::meters;
using namespace webots;
using namespace std;

FeedbackPublisher::FeedbackPublisher(Config* conf, Hardware* hardware, ros::NodeHandle nh,
                                        vector<Sensor_Kinect*>* kinect,
                                        vector<Sensor_Camera*>* camera,
                                        vector<Sensor_Laserscanner*>* laserScanner) {
    pi = conf->pi;
    tf::TransformBroadcaster tfbroad;
    odometryPublisher = nh.advertise<nav_msgs::Odometry>(conf->odometryTopicName, 1);
    pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    this->conf = conf;
    this->hardware = hardware;
    this->kinect = kinect;
    this->camera = camera;
    this->laserScanner = laserScanner;
}

void FeedbackPublisher::run() {
	
	
	const double* getValsPos = hardware->gps[0]->getValues();
    const double* getValsRot = hardware->compass[0]->getValues();
	
    quantity<si::velocity> longitudinalVelocity;
    quantity<si::velocity> transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;

    hardware->youBotBaseKinematic.wheelVelocitiesToCartesianVelocity(hardware->globalWheelVelocities, longitudinalVelocity,
                                                                     transversalVelocity, angularVelocity);

	double rot = getRelativeRotation(getValsRot[0], getValsRot[1], origin_rotation[0], origin_rotation[1]);
	
    double vx_wheel, vy_wheel, vtheta_wheel;
    vx_wheel = longitudinalVelocity.value();
    vy_wheel = transversalVelocity.value();
    vtheta_wheel = angularVelocity.value();

    geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(deg2rad(360-rot));

    geometry_msgs::TransformStamped odometryTransform;
    odometryTransform.header.stamp = ros::Time::now();
    odometryTransform.header.frame_id = conf->odometryTopicName;
    odometryTransform.child_frame_id = "base_footprint";
    odometryTransform.transform.translation.x = (origin_position[0] - getValsPos[0]) * (-1);
    odometryTransform.transform.translation.y = (getValsPos[2] - origin_position[2]) * (-1);
    odometryTransform.transform.translation.z = 0.0;
    odometryTransform.transform.rotation = odometryQuaternion;
	
	// PRINT POSITION OF ROBOT TO CONSOLE
	//cout<<"x: "<< (origin_position[2] - getValsPos[2]) * (-1) << "\ty: "<<(origin_position[0] - getValsPos[0]) * (-1) << "\talpha: "<< 360-rot<<endl;
    tfbroad.sendTransform(odometryTransform);

    nav_msgs::Odometry odometryMessage;
    odometryMessage.header.stamp = ros::Time::now();
    odometryMessage.header.frame_id = conf->odometryTopicName;
    odometryMessage.pose.pose.position.x = (origin_position[0] - getValsPos[0]) * (-1);
    odometryMessage.pose.pose.position.y = (getValsPos[2] - origin_position[2]) * (-1);
    odometryMessage.pose.pose.position.z = 0.0;
    odometryMessage.pose.pose.orientation = odometryQuaternion;
    odometryMessage.child_frame_id = "base_footprint";
    
    // NOT IMPLEMENTED YET
    /*
    odometryMessage.twist.twist.linear.x =  vx_wheel;
    odometryMessage.twist.twist.linear.y = vy_wheel;
    odometryMessage.twist.twist.angular.z = vtheta_wheel;
    */

    odometryPublisher.publish(odometryMessage);
   
for(size_t i = 0; i < laserScanner->size(); i++) {
    if(laserScanner->at(i)->sendTransforms) {
        tfbroad.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromRPY(
                    deg2rad(laserScanner->at(i)->rotation[0]),
                    deg2rad(laserScanner->at(i)->rotation[1]),
                    deg2rad(laserScanner->at(i)->rotation[2])),
                tf::Vector3(laserScanner->at(i)->translation[0],
                            laserScanner->at(i)->translation[1],
                            laserScanner->at(i)->translation[2])),
            ros::Time::now(),
            "base_link",
            laserScanner->at(i)->link_name));

    tfbroad.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromRPY(
                        deg2rad(0),
                        deg2rad(0),
                        deg2rad(0)),
                tf::Vector3(0, 0, 0)),
            ros::Time::now(),
            laserScanner->at(i)->link_name,
            laserScanner->at(i)->topicName));
    }
}

for(size_t i = 0; i < camera->size(); i++) {
    if(camera->at(i)->sendTransforms) {
        tfbroad.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromRPY(
                    deg2rad(camera->at(i)->rotation[0]),
                    deg2rad(camera->at(i)->rotation[1]),
                    deg2rad(camera->at(i)->rotation[2])),
                tf::Vector3(camera->at(i)->translation[0],
                            camera->at(i)->translation[1],
                            camera->at(i)->translation[2])),
            ros::Time::now(),
            "base_link",
            camera->at(i)->link_name));

    tfbroad.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromRPY(
                        deg2rad(0),
                        deg2rad(0),
                        deg2rad(0)),
                tf::Vector3(0, 0, 0)),
            ros::Time::now(),
            camera->at(i)->link_name,
            camera->at(i)->topicName));
    }
}

for(size_t i = 0; i < kinect->size(); i++) {
    if(kinect->at(i)->sendTransforms) {
        tfbroad.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromRPY(
                    deg2rad(kinect->at(i)->rotation[0]),
                    deg2rad(kinect->at(i)->rotation[1]),
                    deg2rad(kinect->at(i)->rotation[2])),
                tf::Vector3(kinect->at(i)->translation[0],
                            kinect->at(i)->translation[1],
                            kinect->at(i)->translation[2])),
            ros::Time::now(),
            "base_link",
            kinect->at(i)->link_name));

    tfbroad.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromRPY(
                        deg2rad(0),
                        deg2rad(0),
                        deg2rad(0)),
                tf::Vector3(0, 0, 0)),
            ros::Time::now(),
            kinect->at(i)->link_name,
            kinect->at(i)->topicName));
    }
}


    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(7);
    joint_state.name[0] = "arm_joint_1";
    joint_state.name[1] = "arm_joint_2";
    joint_state.name[2] = "arm_joint_3";
    joint_state.name[3] = "arm_joint_4";
    joint_state.name[4] = "arm_joint_5";
    joint_state.name[5] = "gripper_finger_joint_l";
    joint_state.name[6] = "gripper_finger_joint_r";
    joint_state.position.resize(7);
    joint_state.position[0] = armPosFromWebots(0);
    joint_state.position[1] = armPosFromWebots(1);
    joint_state.position[2] = armPosFromWebots(2);
    joint_state.position[3] = armPosFromWebots(3);
    joint_state.position[4] = armPosFromWebots(4);
    joint_state.position[5] = hardware->handGripper[0]->getPosition();
    joint_state.position[6] = hardware->handGripper[1]->getPosition();

    joint_state.velocity.resize(7);
    for (int i = 0; i < 4; i++) {
        joint_state.velocity[i] = 0;
    }


    // send the joint state and transform
    pub_joint_states.publish(joint_state);

    // wheel joint_states
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.velocity.resize(8);
    joint_state.name[0] = "wheel_joint_fl";
    joint_state.name[1] = "wheel_joint_fr";
    joint_state.name[2] = "wheel_joint_bl";
    joint_state.name[3] = "wheel_joint_br";
    joint_state.name[4] = "caster_joint_fl";
    joint_state.name[5] = "caster_joint_fr";
    joint_state.name[6] = "caster_joint_bl";
    joint_state.name[7] = "caster_joint_br";
    joint_state.position[0] = hardware->wheel[0]->getPosition();
    joint_state.position[1] = hardware->wheel[1]->getPosition();
    joint_state.position[2] = hardware->wheel[2]->getPosition();
    joint_state.position[3] = hardware->wheel[3]->getPosition();
    joint_state.position[4] = 0;
    joint_state.position[5] = 0;
    joint_state.position[6] = 0;
    joint_state.position[7] = 0;
    joint_state.velocity[0] = hardware->globalWheelVelocities[0].value();
    joint_state.velocity[1] = hardware->globalWheelVelocities[1].value();
    joint_state.velocity[2] = hardware->globalWheelVelocities[2].value();
    joint_state.velocity[3] = hardware->globalWheelVelocities[3].value();
    joint_state.velocity[4] = 0;
    joint_state.velocity[5] = 0;
    joint_state.velocity[6] = 0;
    joint_state.velocity[7] = 0;
    // send the joint state and transform
    pub_joint_states.publish(joint_state);
}

double FeedbackPublisher::deg2rad(double deg) {
    return deg * (pi/180.0);
}

float FeedbackPublisher::armPosFromWebots(int arm) {
    switch (arm) {
        case(0):
        return armPosFromWebots(arm, hardware->armJoint[0]->getPosition());
        break;
    case(1):
        return armPosFromWebots(arm, hardware->armJoint[1]->getPosition());
        break;
    case(2):
        return armPosFromWebots(arm, hardware->armJoint[2]->getPosition());
        break;
    case(3):
        return armPosFromWebots(arm, hardware->armJoint[3]->getPosition());
        break;
    case(4):
        return armPosFromWebots(arm, hardware->armJoint[4]->getPosition());
        break;
    default:
        return armPosFromWebots(arm, NAN);
        break;
    }
}

float FeedbackPublisher::armPosFromWebots(int arm, float webotsArmPos) {
    webotsArmPos *= -1;
    switch (arm) {
        case(0):
            webotsArmPos -= conf->armJoint0Offset;
            break;
        case(1):
            webotsArmPos -= conf->armJoint1Offset;
            break;
        case(2):
            webotsArmPos -= conf->armJoint2Offset;
            break;
        case(3):
            webotsArmPos -= conf->armJoint3Offset;
            break;
        case(4):
            webotsArmPos -= conf->armJoint4Offset;
            break;
        default:
            cerr << "Joint with index " << arm << " does not exist!" << endl;
            return NAN;
        break;
    }
    return webotsArmPos;
}

void FeedbackPublisher::initiatePositions() {
	const double* getVals = hardware->gps[0]->getValues();
	origin_position.push_back(getVals[0]);
	origin_position.push_back(getVals[1]);
	origin_position.push_back(getVals[2]);

    getVals = hardware->compass[0]->getValues();

	origin_rotation.push_back(getVals[0]);
	origin_rotation.push_back(getVals[1]);
	origin_rotation.push_back(getVals[2]);
}

double FeedbackPublisher::getRelativeRotation(double xaxis, double yaxis, 
											   double origin_xaxis, double origin_yaxis) {
	double relativeRot = 0;
	double rot = 90 * abs(xaxis - 1);
	if(yaxis > 0) {
		rot = 0 + rot;
	}else {
		rot = 360 - rot;
	}
	
	double origin_rot = 90 * abs(origin_xaxis - 1);
	if(origin_yaxis > 0) {
		origin_rot = 0 + origin_rot;
	}else {
		origin_rot = 360 - origin_rot;
	}
	
	relativeRot = rot - origin_rot;
	if(relativeRot < 0) relativeRot = relativeRot + 360;
	
	return relativeRot;
}
	

