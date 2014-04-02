/*
 * File:          sensor_kinect.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

/*
 * controller includes
 */
#include "sensor_kinect.h"
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

/*
 * namespaces
 */
using namespace webots;
using namespace std;

Sensor_Kinect::Sensor_Kinect(Config* conf, Hardware* hardware, ros::NodeHandle nh, int index) {
    width = hardware->kinect[index]->getWidth();
    height = hardware->kinect[index]->getHeight();
    header_kinect_seq = 0;
    link_name = "base_"+conf->devDetails[1][index].topicName+"_link";
    topicName = "/"+conf->devDetails[1][index].topicName;

    sensor_msgs::PointField fieldX, fieldY, fieldZ;
    fieldX.name = "x";
    fieldX.datatype = 7;  // FLOAT32
    fieldX.offset = 0;
    fieldX.count = 1;
    fields.push_back(fieldX);

    fieldY.name = "y";
    fieldY.datatype = 7;  // FLOAT32
    fieldY.offset = sizeof(float)*1;
    fieldY.count = 1;
    fields.push_back(fieldY);

    fieldZ.name = "z";
    fieldZ.datatype = 7;  // FLOAT32
    fieldZ.offset = sizeof(float)*2;
    fieldZ.count = 1;
    fields.push_back(fieldZ);

    x = new float[width*height];  // point cloud x values
    y = new float[width*height];  // point cloud y values
    z = new float[width*height];  // point cloud z values

    // camera intrinsic parameters, representative values, see http://nicolas.burrus.name/index.php/Research/KinectCalibration for more info
    cx = width/2.0;  // center of projection
    cy = height/2.0;  // center of projection
    fx = 300.0;  // focal length in pixels
    fy = 300.0;  // focal length in pixels

    data_size = width*height*sizeof(float)*4;

    this->conf = conf;
    this->hardware = hardware;
    this->index = index;

    sendTransforms = conf->devDetails[1][index].sendTransforms;
    for (int i = 0; i < 3; i++) {
        rotation[i] = conf->devDetails[1][index].rotation[i];
    }
    for (int i = 0; i < 3; i++) {
        translation[i] = conf->devDetails[1][index].translation[i];
    }

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(topicName, 1000);
}

Sensor_Kinect::~Sensor_Kinect() {
    delete[] x;
    delete[] y;
	delete[] z;
}

void Sensor_Kinect::run() {
    kinectValues = hardware->kinect[index]->getRangeImage();

    for (size_t v = 0, n = 0; v < height; v++) {
        for (size_t u = 0; u < width; u++, n++) {
            float depth = hardware->kinect[index]->rangeImageGetDepth(kinectValues, width, u, v);
            x[n] = (u - cx) * depth / fx;
            y[n] = (v - cy) * depth / fy;
            z[n] = depth;
        }
    }

    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.seq = header_kinect_seq++;
    pc_msg.header.frame_id = topicName;

    pc_msg.width = width;
    pc_msg.height = height;
    pc_msg.fields = fields;

    pc_msg.is_bigendian = false;
    pc_msg.is_dense = false;
    pc_msg.point_step = 16;
    pc_msg.row_step = pc_msg.point_step * width;

    pc_msg.data.resize(data_size);

    valuesX = reinterpret_cast<uint8_t*>(x);
    valuesY = reinterpret_cast<uint8_t*>(y);
    valuesZ = reinterpret_cast<uint8_t*>(z);
    for (size_t i = 0; i < data_size; i += pc_msg.point_step) {
        for (size_t j = 0; j < sizeof(float); j++) {
            pc_msg.data[i+j] = *(valuesX++);
        }
        for (size_t j = sizeof(float)*1; j < sizeof(float)*2; j++) {
            pc_msg.data[i+j] = *(valuesY++);
        }
        for (size_t j = sizeof(float)*2; j < sizeof(float)*3; j++) {
            pc_msg.data[i+j] = *(valuesZ++);
        }
        for (size_t j = sizeof(float)*3; j < sizeof(float)*4; j++) {
            pc_msg.data[i+j] = 0;
        }
    }
    pub_cloud.publish(pc_msg);
}
