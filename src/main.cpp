/*
 * File:          main.cpp
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */
 

/*
 * controller includes
 */
#include "world.h"

/*
 * c++ includes
 */
#include <iostream>

/*
 * namespaces
 */
using namespace std;

int main(int argc, char **argv) {

	cout<<"II: Creating World....";
	World *world = new World();
	cout<<"\t\t done"<<endl;
	world->run(argc, argv);
	
	delete world;
    return 0;
}



