youBot ROS-Controller for Webots
=============


Introduction
------------------
This controller is used for simulation purposes while debugging and developing code for
the youBot from KUKA Robotics. It publishes and subscribes to all topics just like the
youbot_driver package from Locomotec is doing it.


Usage
------------------
* start webots via "roslaunch youbot_webots_ros_controller webots.launch"
* load "Example.wbt" from worlds directory
* use topics like you would use them while they are offered by the real youbot_driver


Notes
------------------
* The example world project contains an invisible camera attached to the gripper additionally to the kinect
* For the odometry the controller needs a compass and a gps receiver attached to it's base link
* Due to incompatibilities of newer Webot versions proto-files, the given proto-files are compatible up to version 7.1.2


Dependencies
------------------
* ROS-Environment
* Webots Pro to compile the controller
