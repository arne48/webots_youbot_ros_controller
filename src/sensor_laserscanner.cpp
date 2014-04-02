/*
 * File:          sensor_laserscanner.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

/*
 * controller includes
 */
#include "config.h"
#include "sensor_laserscanner.h"

/*
 * ros includes
 */
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

/*
 * c++ includes
 */
#include <string>
#include <vector>

/*
 * namespaces
 */ 
using namespace webots;
using namespace std;

Sensor_Laserscanner::Sensor_Laserscanner(Config* conf, Hardware* hardware, ros::NodeHandle nh, int index) {
    header_seq = 0;
    link_name = "base_"+conf->devDetails[0][index].topicName+"_link";
    topicName = "/"+conf->devDetails[0][index].topicName;
    laser_samples = hardware->laserScanner[index]->getWidth();
    laser_fov = hardware->laserScanner[index]->getFov();
    angle_min_map = conf->angle_min_map;
    angle_max_map = conf->angle_max_map;
    angle_increment = (laser_fov - conf->oneDegRad) / laser_samples;
    angle_min_intern = -laser_fov / 2.0;
    angle_max_intern = laser_fov / 2.0;
    angle_min_i = abs((angle_min_intern - angle_min_map) / angle_increment);
    angle_max_i = abs((angle_max_intern + angle_max_map) / angle_increment);
    angle_min = angle_min_intern + angle_min_i * angle_increment;
    angle_max = angle_min_intern + angle_max_i * angle_increment;
    laserPublisher = nh.advertise<sensor_msgs::LaserScan>(topicName, 1000);
    this->index = index;
    this->conf = conf;
    this->hardware = hardware;
    sendTransforms = conf->devDetails[0][index].sendTransforms;
    for (int i = 0; i < 3; i++) {
        rotation[i] = conf->devDetails[0][index].rotation[i];
    }
    for (int i = 0; i < 3; i++) {
        translation[i] = conf->devDetails[0][index].translation[i];
    }
}

void Sensor_Laserscanner::run() {
    laser_values  = hardware->laserScanner[index]->getRangeImage();
    scan_msg.header.stamp     = ros::Time::now();
    scan_msg.header.seq       = header_seq++;
    scan_msg.header.frame_id  = topicName;
    scan_msg.angle_min        = angle_min;
    scan_msg.angle_max        = angle_max;
    scan_msg.angle_increment  = angle_increment;
    scan_msg.time_increment   = conf->laserTimeIncrement;
    scan_msg.scan_time        = conf->laserScanTime;
    scan_msg.range_min        = conf->laserMinRange;
    scan_msg.range_max        = conf->laserMaxRange;
    scan_msg.ranges           = vector<float>(laser_samples);

    for (size_t s = angle_min_i; s < angle_max_i; s++) {
        if (laser_values[s] > (conf->laserMaxRange - 0.03)) {
            scan_msg.ranges[angle_max_i-s-1]  = 0;
        } else {
            scan_msg.ranges[angle_max_i-s-1]  = laser_values[s];
        }
    }
    laserPublisher.publish(scan_msg);
}
