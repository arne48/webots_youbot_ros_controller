/*
 * File:          sensor_camera.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef  CONTROLLERS_YOUBOT_SENSOR_CAMERA_H_
#define  CONTROLLERS_YOUBOT_SENSOR_CAMERA_H_

/*
 * controller includes
 */
#include "config.h"
#include "hardware.h"

/*
 * ros includes
 */
#include "ros/ros.h"
#include <image_transport/image_transport.h>

/*
 * c++ includes
 */
#include <string>


class Sensor_Camera {
    public:
        Sensor_Camera(Config* conf, Hardware* hardware, ros::NodeHandle nh, int index);
        void run();
        std::string topicName;
        std::string link_name;
        bool sendTransforms;
        double rotation[3];
        double translation[3];
    private:
        Config* conf;
        Hardware* hardware;
        int index;
        unsigned int seq;
        image_transport::Publisher pub_image;
        const unsigned char *image;
};
#endif  // CONTROLLERS_YOUBOT_SENSOR_CAMERA_H_

