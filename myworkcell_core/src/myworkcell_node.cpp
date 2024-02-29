#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelState.h"
#include <gripper_controls/gripper.h>
#include "pick_and_place_control/pick_and_place.h"
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include "aricc_2d_vision/rotate_object_detection.h"
#include "aricc_utils/geometry_utils.h"
#define _USE_MATH_DEFINES
#include <cmath>
#define DXL_LEFT 1
#define DXL_RIGHT 2
#define ADDRESS "Goal_Position"


// class ScanNPlan
// {
// public:
//   ScanNPlan(ros::NodeHandle& nh) : nh_(nh), gripper(nh)
//   {
//     vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
//   }

//   void start(const std::string& base_frame)
//   {
//     ROS_INFO("Attempting to localize part");

//     double z_offset_above_object = getParameter("z_offset_above_object", 0.2);
//     double z_offset_on_object = getParameter("z_offset_on_object", 0.1);

//     // Localize the part
//     ros::Duration(1.0).sleep();
//     myworkcell_core::LocalizePart srv;
//     srv.request.base_frame = base_frame;

//     ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

//     if (!vision_client_.call(srv))
//     {
//       ROS_ERROR("Could not localize part");
//       return;
//     }
//     ROS_INFO_STREAM("Part localized: " << srv.response);

//     geometry_msgs::Pose move_target = srv.response.pose;

//     move_target.position.z = z_offset_above_object;
//     move_target.position.x -=0.005; // + is move up , - is go down
//     move_target.position.y -= 0.005; // - is going right , + is going left

//     ROS_INFO_STREAM("move_target: " << move_target);
//     moveit::planning_interface::MoveGroupInterface move_group("gluon_arm");
//     move_group.setGoalJointTolerance(0.001);

//     gripper.open();
//     // Plan for the robot to move to the part
//     move_group.setPlanningTime(10);
//     move_group.setPoseReferenceFrame(base_frame);
//     move_group.setPoseTarget(move_target);
//     move_group.move();

//     move_target.position.z = z_offset_on_object;


//     move_group.setPoseTarget(move_target);
//     move_group.move();

//     // Additional actions...

//     gripper.closeHalf();

//     move_group.setNamedTarget("look_table_pose1");
//     move_group.move();
//     move_group.setNamedTarget("approach_tray");
//     move_group.move();
//     move_group.setNamedTarget("approach_tray_three");
//     move_group.move();
//     move_group.setNamedTarget("tray_three");
//     move_group.move();

//   gripper.open();

//     ROS_INFO("Arm is moving to look_table_pose1 position");

//     move_group.setNamedTarget("approach_tray_three");
//     move_group.move();
//     move_group.setNamedTarget("approach_tray");
//     move_group.move();
//     move_group.setNamedTarget("look_table_pose1");
//     move_group.move();
//   }

// private:
//   ros::ServiceClient vision_client_;
//   ros::NodeHandle nh_;
//   Gripper gripper;

//   double getParameter(const std::string& param_name, double default_value)
//   {
//     double param_value;
//     if (!nh_.getParam(param_name, param_value)) {
//       ROS_WARN("Parameter '%s' not found. Using default value.", param_name.c_str());
//       param_value = default_value;
//     }
//     return param_value;
//   }
// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "myworkcell_node");
//   ros::NodeHandle nh;
//   ros::NodeHandle private_node_handle("~");
//   ros::AsyncSpinner async_spinner(1);
//   async_spinner.start();
//   ROS_INFO("ScanNPlan node has been initialized");
//   ScanNPlan app(nh);
//   std::string base_frame;
//   private_node_handle.param<std::string>("base_frame", base_frame, "world");
//   ros::Duration(0.5).sleep();
//   app.start(base_frame);
//   ros::waitForShutdown();
// }

// 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "myworkcell_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    PickAndPlace pick_and_place(nh);
    // Define the loop rate (e.g., 1 Hz)
    // ros::spinOnce();  // Process callbacks
    ros::Duration(5.0).sleep();  // Introduce a 10-second delay
    pick_and_place.transformation();  // Run transformation
    pick_and_place.moveit();  // Run MoveIt

    ros::waitForShutdown();
    return 0;
}

