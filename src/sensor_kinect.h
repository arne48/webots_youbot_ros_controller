/*
 * File:          sensor_kinect.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef CONTROLLERS_YOUBOT_SENSOR_KINECT_H_
#define CONTROLLERS_YOUBOT_SENSOR_KINECT_H_
/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"

/*
 * ros includes
 */
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

/*
 * c++ includes
 */
#include <string>
#include <vector>

class Sensor_Kinect {
    public:
        Sensor_Kinect(Config* conf, Hardware* hardware, ros::NodeHandle nh, int index);
        ~Sensor_Kinect();
        void run();
        std::string topicName;
        std::string link_name;
        bool sendTransforms;
        double rotation[3];
        double translation[3];

    private:
        ros::Publisher pub_cloud;
        size_t width;
        size_t height;
        std::vector<sensor_msgs::PointField> fields;
        const float* kinectValues;
        sensor_msgs::PointCloud2 pc_msg;
        float* x;
        float* y;
        float* z;
        float cx;
        float cy;
        float fx;
        float fy;
        int i;
        size_t data_size;
        unsigned int header_kinect_seq;
        std::string frame_name;
        uint8_t* valuesX;
        uint8_t* valuesY;
        uint8_t* valuesZ;
        int index;
        Config* conf;
        Hardware* hardware;
};
#endif  // CONTROLLERS_YOUBOT_SENSOR_KINECT_H_
