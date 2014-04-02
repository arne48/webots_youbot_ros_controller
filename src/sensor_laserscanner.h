/*
 * File:          sensor_laserscanner.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef CONTROLLERS_YOUBOT_SENSOR_LASERSCANNER_H_
#define CONTROLLERS_YOUBOT_SENSOR_LASERSCANNER_H_

/*
 * ros includes
 */
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"

/*
 * c++ includes
 */
#include <string>

class Sensor_Laserscanner {
    public:
        Sensor_Laserscanner(Config* conf, Hardware* hardware, ros::NodeHandle nh, int index);
        void run();
        std::string topicName;
        std::string link_name;
        bool sendTransforms;
        double rotation[3];
        double translation[3];

    private:
        int index;
        unsigned int header_seq;
        const float* laser_values;
        int laser_samples;
        double laser_fov;
        double angle_min_map;
        double angle_max_map;
        double angle_increment;
        double angle_min_intern;
        double angle_max_intern;
        size_t angle_min_i;
        size_t angle_max_i;
        double angle_min;
        double angle_max;
        ros::Publisher laserPublisher;
        sensor_msgs::LaserScan scan_msg;
        Config* conf;
        Hardware* hardware;
};
#endif  // CONTROLLERS_YOUBOT_SENSOR_LASERSCANNER_H_
