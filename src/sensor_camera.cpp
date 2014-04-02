/*
 * File:          sensor_camera.cpp
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
#include "sensor_camera.h"

/*
 * c++ includes
 */
#include <vector>

/*
 * ros includes
 */
#include "ros/ros.h"
#include <image_transport/image_transport.h>

Sensor_Camera::Sensor_Camera(Config* conf, Hardware* hardware, ros::NodeHandle nh, int index) {
    link_name = "base_"+conf->devDetails[2][index].topicName+"_link";
    topicName = "/"+conf->devDetails[2][index].topicName;
    image_transport::ImageTransport it(nh);
    pub_image = it.advertise(topicName, 1);
    seq = 0;
    this->conf = conf;
    this->hardware = hardware;
    this->index = index;
    sendTransforms = conf->devDetails[2][index].sendTransforms;
    for (int i = 0; i < 3; i++) {
        rotation[i] = conf->devDetails[2][index].rotation[i];
    }
    for (int i = 0; i < 3; i++) {
        translation[i] = conf->devDetails[2][index].translation[i];
    }
}

void Sensor_Camera::run() {
    image = hardware->camera[index]->getImage();
    sensor_msgs::Image msg;
    msg.header.seq = seq;
    msg.header.stamp = ros::Time::now();
    msg.height = hardware->camera[index]->getHeight();
    msg.width = hardware->camera[index]->getWidth();
    msg.encoding = "bgra8";
    std::vector<unsigned char> imgdata(image, image + msg.height * msg.width * 4);
    msg.step = msg.width * 4;
    msg.is_bigendian = 0;
    msg.data = imgdata;
    seq++;
    pub_image.publish(msg);
}
