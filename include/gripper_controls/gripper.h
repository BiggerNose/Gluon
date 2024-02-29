/* gripper.h */

#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>

class Gripper {
public:
    Gripper(ros::NodeHandle& nh); // Constructor

    bool open(); // Function to open the gripper
    bool release(); // Function to release the gripper
    bool closeTight(); // Function to close the gripper
    bool closeHalf(); // Function to close the gripper
    bool sweepLeft(); // Function to close the gripper
    bool sweepRight(); // Function to close the gripper
    bool closeGripper(const std::string );
    bool closeLight();

private:
    ros::NodeHandle node_handle;
    std::string object_name ;
};

#endif // GRIPPER_H
