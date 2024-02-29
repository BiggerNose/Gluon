#ifndef PICK_AND_PLACE_H
#define PICK_AND_PLACE_H

#include <ros/ros.h>
#include "aricc_vision_msgs/ObjectArray.h"
#include "aricc_vision_msgs/Object.h"
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include "aricc_utils/geometry_utils.h"
#include "aricc_2d_vision/rotate_object_detection.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include<moveit/robot_state/robot_state.h>



class PickAndPlace {
public:
    PickAndPlace(ros::NodeHandle& nh ); // Constructor
    void objectCallback(const aricc_vision_msgs::ObjectArray::ConstPtr& msg);
    void visionCallback(const aricc_vision_msgs::RotateObjectArray::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg);
    bool transformation();
    geometry_msgs::Pose move_target;
    bool moveit();
    bool lookup();
    ros::Subscriber pixel_image_sub; 
    ros::Subscriber depth_image_sub;
    ros::Subscriber object_sub;
private:
    ros::NodeHandle node_handle;
    aricc_vision_msgs::RotateObject last_object; 
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_; 
    double depth_value;
    cv::Point2d last_position_;
    geometry_msgs::Pose target_pose_from_cam; 
    moveit::planning_interface::MoveGroupInterface move_group; 
    cv::Mat depth_image; 
    aricc_vision_msgs::Object last_object_; 
    // geometry_msgs::Pose move_target;


    // Declare other private members if needed
};

#endif // PICK_AND_PLACE_H