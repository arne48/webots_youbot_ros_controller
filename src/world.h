/*
 * File:          world.h
 * Date:          29.03.2013
 * Description:   Webots ROS Connection 
 * Copyright:     GPLv2
 * Author:        arne.hitzmann@ostfalia.de
 */

#ifndef  CONTROLLERS_YOUBOT_WORLD_H_
#define  CONTROLLERS_YOUBOT_WORLD_H_
/*
 * webots includes
 */
#include <webots/Robot.hpp>


class World : public webots::Robot {
    public:
		void run(int argc, char **argv);
	private:
};
#endif  // CONTROLLERS_YOUBOT_WORLD_H_
