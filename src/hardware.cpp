/*
 * File:          hardware.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

/*
 * controller includes
 */
#include "hardware.h"

/*
 * webots includes
 */
#include <webots/Robot.hpp>
#include <webots/Servo.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

/*
 * c++ includes
 */
#include <string>
#include <vector>
#include <iostream>

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
 * namespaces
 */
using namespace boost::units;
using namespace boost::units::si;
using namespace boost::units::angle;
using boost::units::si::meters;
using namespace webots;
using namespace std;

Hardware::Hardware(Config* conf) {
	cout<<"\nII: adding wheels to hardware setup...";
    wheel.push_back(new webots::Servo(conf->wheelNames[0]));
    wheel.push_back(new webots::Servo(conf->wheelNames[1]));
    wheel.push_back(new webots::Servo(conf->wheelNames[2]));
    wheel.push_back(new webots::Servo(conf->wheelNames[3]));
	cout<<"\t\t done"<<endl;

    cout<<"II: adding arm-joints to hardware setup...";
    armJoint.push_back(new webots::Servo(conf->armNames[0]));
    armJoint.push_back(new webots::Servo(conf->armNames[1]));
    armJoint.push_back(new webots::Servo(conf->armNames[2]));
    armJoint.push_back(new webots::Servo(conf->armNames[3]));
    armJoint.push_back(new webots::Servo(conf->armNames[4]));
    cout<<"\t\t done"<<endl;

	cout<<"II: adding gripper-joints to hardware setup...";
    handGripper.push_back(new webots::Servo(conf->fingerNames[0]));
    handGripper.push_back(new webots::Servo(conf->fingerNames[1]));
    cout<<"\t done"<<endl;

	cout<<"II: adding laserscanners to hardware setup...";
    for (size_t i = 0; i < conf->devDetails[0].size(); i++) {
        laserScanner.push_back(new webots::Camera(conf->devDetails[0][i].name));
    }
    cout<<"\t\t done"<<endl;

	cout<<"II: adding kinect-sensors to hardware setup...";
    for (size_t i = 0; i < conf->devDetails[1].size(); i++) {
        kinect.push_back(new webots::Camera(conf->devDetails[1][i].name));
    }
    cout<<"\t done"<<endl;

	cout<<"II: adding cameras to hardware setup...";
    for (size_t i = 0; i < conf->devDetails[2].size(); i++) {
        camera.push_back(new webots::Camera(conf->devDetails[2][i].name));
    }
    cout<<"\t\t done"<<endl;

	gps.push_back(new webots::GPS("gps"));
	gps[0]->enable(conf->timeStep);
	compass.push_back(new webots::Compass("compass"));
	compass[0]->enable(conf->timeStep);

	cout<<"II: activating arm-joints...";
    armJoint[0]->enablePosition(conf->timeStep);
    armJoint[1]->enablePosition(conf->timeStep);
    armJoint[2]->enablePosition(conf->timeStep);
    armJoint[3]->enablePosition(conf->timeStep);
    armJoint[4]->enablePosition(conf->timeStep);
	cout<<"\t\t done"<<endl;

	cout<<"II: activating gripper-joints...";
    handGripper[0]->enablePosition(conf->timeStep);
    handGripper[1]->enablePosition(conf->timeStep);
	cout<<"\t\t done"<<endl;

	cout<<"II: activating wheels...";
    wheel[0]->enablePosition(conf->timeStep);
    wheel[1]->enablePosition(conf->timeStep);
    wheel[2]->enablePosition(conf->timeStep);
    wheel[3]->enablePosition(conf->timeStep);
    cout<<"\t\t\t done"<<endl;

	cout<<"II: activating laserscanners...";
    for (size_t i = 0; i < laserScanner.size(); i++) {
        laserScanner[i]->enable(conf->timeStep);
    }
    cout<<"\t\t done"<<endl;

	cout<<"II: activating kinectsensors...";
    for (size_t i = 0; i < kinect.size(); i++) {
        kinect[i]->enable(conf->timeStep);
    }
    cout<<"\t\t done"<<endl;

	cout<<"II: activating cameras...";
    for (size_t i = 0; i < camera.size(); i++) {
        camera[i]->enable(conf->timeStep);
    }
    cout<<"\t\t\t done"<<endl;
    

	cout<<"II: initializing velocity storage...";
    for (int i = 0; i < 4; i++) {
        globalWheelVelocities.push_back(0 * radian_per_second);
    }
    cout<<"\t\t done"<<endl;

	cout<<"II: initializing kinematics...";
    youbot::FourSwedishWheelOmniBaseKinematicConfiguration youBotBaseKinematicConfig;
    youBotBaseKinematicConfig.rotationRatio = conf->baseRotationRatio;
    youBotBaseKinematicConfig.slideRatio = conf->baseSlideRatio;
    youBotBaseKinematicConfig.lengthBetweenFrontAndRearWheels = conf->baseDistFrontRearWheels;
    youBotBaseKinematicConfig.lengthBetweenFrontWheels = conf->baseDistFrontWheels;
    youBotBaseKinematicConfig.wheelRadius = conf->baseWheelRadius;
    youBotBaseKinematic.setConfiguration(youBotBaseKinematicConfig);
    cout<<"\t\t done"<<endl;
}
