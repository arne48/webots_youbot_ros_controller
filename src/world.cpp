/*
 * File:          world.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */
 
 #define WAITSEC 5

/*
 * controller includes
 */
#include "youbot.h"
#include "world.h"

/*
 * c++ includes
 */
#include <sys/time.h>
#include <unistd.h>

/*
 * ros includes
 */
#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>

/*
 * namespaces
 */
using namespace webots;
using namespace std;

void World::run(int argc, char **argv) {
    /*
     * ROS Init
     */ 
    cout<<"II: Initializing ROS...";
    ros::init(argc, argv, "webots_sim");
    cout<<"\t\t done"<<endl;
    cout<<"II: Creating ROS-Handle...";
    ros::NodeHandle nh;
    cout<<"\t\t done"<<endl;
    cout<<"II: >>Initializing Robot";
    YouBot *robot = new YouBot(nh);
    cout<<"II: <<Robot initialized"<<endl;
    cout<<"II: Initializing Simulation Clock...";
    ros::Publisher timePublisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1000);
    rosgraph_msgs::Clock time_msg;
    ros::Time simTime = ros::Time::now();
    ros::Duration addStep(0.016);
    
    cout<<"\t\t done"<<endl;
  
	cout<< "II: Initiating Odometry...";
	// QUICK-FIX BEGIN
	// Wait some time for the robot to sattle
	struct timeval start, end;
	long mtime, seconds, useconds;   
	gettimeofday(&start, NULL);
	do{
		gettimeofday(&end, NULL);
		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
		step(16);
	}while(mtime <= WAITSEC * 1000);
	// QUICK-FIX END
	robot->initiatePositions();
	cout<<"\t\t\t done"<<endl;
	
	
	cout<<"II: Starting Simulation Loop..."<<endl;
	while (step(16) != -1) {
			// Run Robot Routines
			robot->run();
			
			// Update Time of ROS
			simTime = simTime + addStep;
			time_msg.clock = simTime;
			timePublisher.publish(time_msg);
	}
    delete robot;
}



