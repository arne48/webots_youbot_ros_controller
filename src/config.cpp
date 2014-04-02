/*
 * File:          config.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

/*
 * controller includes
 */
#include "config.h"

/*
 * ros includes
 */
#include <tf/transform_broadcaster.h>

/*
 * xml includes
 */
#include <tinyxml.h>

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
#include <iostream>
#include <fstream>
#include <unistd.h>

/*
 * namespaces
 */
using namespace boost::units;
using namespace boost::units::si;
using namespace boost::units::angle;
using boost::units::si::meters;
using namespace std;


Config::Config(std::string path) {
    TiXmlDocument doc;
    doc.LoadFile(path.c_str());
    TiXmlElement* root = doc.RootElement();

    timeStep = atoi(root->FirstChildElement("timeStep")->GetText());
    baseJoints = atoi(root->FirstChildElement("baseJoints")->GetText());
    baseSlideRatio = atof(root->FirstChildElement("baseSlideRatio")->GetText());
    baseRotationRatio = atof(root->FirstChildElement("baseRotationRatio")->GetText());
    pi = atof(root->FirstChildElement("pi")->GetText());
    oneDegRad = atof(root->FirstChildElement("oneDegRad")->GetText());
    laserTimeIncrement = atof(root->FirstChildElement("laserTimeIncrement")->GetText());
    laserScanTime = atof(root->FirstChildElement("laserScanTime")->GetText());
    laserMinRange = atof(root->FirstChildElement("laserMinRange")->GetText());
    laserMaxRange = atof(root->FirstChildElement("laserMaxRange")->GetText());
    armJoint0Offset = atof(root->FirstChildElement("armJoint0Offset")->GetText());
    armJoint1Offset = atof(root->FirstChildElement("armJoint1Offset")->GetText());
    armJoint2Offset = atof(root->FirstChildElement("armJoint2Offset")->GetText());
    armJoint3Offset = atof(root->FirstChildElement("armJoint3Offset")->GetText());
    armJoint4Offset = atof(root->FirstChildElement("armJoint4Offset")->GetText());
    angle_max_map = atof(root->FirstChildElement("angle_max_map")->GetText());
    angle_min_map = atof(root->FirstChildElement("angle_min_map")->GetText());
    odometryTopicName = root->FirstChildElement("odometryTopicName")->GetText();
    baseDistFrontRearWheels = atof(root->FirstChildElement("baseDistFrontRearWheels")->GetText()) * meter;
    baseDistFrontWheels = atof(root->FirstChildElement("baseDistFrontWheels")->GetText()) * meter;
    baseWheelRadius = atof(root->FirstChildElement("baseWheelRadius")->GetText()) * meter;

    TiXmlElement* child;
    vector<device> devices;

    child = root->FirstChildElement("wheels");
    for (TiXmlNode* pChild = child->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
       wheelNames.push_back(pChild->ToElement()->GetText());
    }

    child = root->FirstChildElement("arm");
    for (TiXmlNode* pChild = child->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
       armNames.push_back(pChild->ToElement()->GetText());
    }

    child = root->FirstChildElement("fingers");
    for (TiXmlNode* pChild = child->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
       fingerNames.push_back(pChild->ToElement()->GetText());
    }

    child = root->FirstChildElement("laserScanner");
    for (TiXmlNode* pChild = child->FirstChild("device"); pChild != 0; pChild = pChild->NextSibling( "device" )) {
        struct device dev;
        dev.name = pChild->ToElement()->Attribute("name");
        dev.topicName = pChild->ToElement()->FirstChildElement("topicName")->GetText();

        string str = pChild->ToElement()->FirstChildElement("sendTransforms")->GetText();
        if(str == "true") {
            dev.sendTransforms = true;
        }else{
            dev.sendTransforms = false;
        }

        dev.transformReference = pChild->ToElement()->FirstChildElement("transformReference")->GetText();

        dev.rotation[0] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("x"));
        dev.rotation[1] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("y"));
        dev.rotation[2] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("z"));

        dev.translation[0] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("x"));
        dev.translation[1] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("y"));
        dev.translation[2] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("z"));
        devices.push_back(dev);
    }

    devDetails.push_back(devices);
    devices.clear();

    child = root->FirstChildElement("kinect");
    for (TiXmlNode* pChild = child->FirstChild("device"); pChild != 0; pChild = pChild->NextSibling("device")) {
        struct device dev;
        dev.name = pChild->ToElement()->Attribute("name");
        dev.topicName = pChild->ToElement()->FirstChildElement("topicName")->GetText();

        string str = pChild->ToElement()->FirstChildElement("sendTransforms")->GetText();
        if(str == "true") {
            dev.sendTransforms = true;
        }else{
            dev.sendTransforms = false;
        }

        dev.transformReference = pChild->ToElement()->FirstChildElement("transformReference")->GetText();

        dev.rotation[0] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("x"));
        dev.rotation[1] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("y"));
        dev.rotation[2] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("z"));

        dev.translation[0] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("x"));
        dev.translation[1] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("y"));
        dev.translation[2] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("z"));
        devices.push_back(dev);
    }

    devDetails.push_back(devices);
    devices.clear();

    child = root->FirstChildElement("camera");
    for ( TiXmlNode* pChild = child->FirstChild("device"); pChild != 0; pChild = pChild->NextSibling("device")) {
        struct device dev;
        dev.name = pChild->ToElement()->Attribute("name");
        dev.topicName = pChild->ToElement()->FirstChildElement("topicName")->GetText();

        string str = pChild->ToElement()->FirstChildElement("sendTransforms")->GetText();
        if(str == "true") {
            dev.sendTransforms = true;
        }else{
            dev.sendTransforms = false;
        }

        dev.transformReference = pChild->ToElement()->FirstChildElement("transformReference")->GetText();

        dev.rotation[0] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("x"));
        dev.rotation[1] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("y"));
        dev.rotation[2] = atof(pChild->ToElement()->FirstChildElement("rotation")->Attribute("z"));

        dev.translation[0] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("x"));
        dev.translation[1] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("y"));
        dev.translation[2] = atof(pChild->ToElement()->FirstChildElement("translation")->Attribute("z"));
        devices.push_back(dev);
    }
    devDetails.push_back(devices);
    devices.clear();
}





