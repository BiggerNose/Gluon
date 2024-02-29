#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "aricc_vision_msgs/ObjectArray.h"
#include "aricc_vision_msgs/Object.h"
// #include <tf2/convert.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/transform_datatypes.h>



void poseCallback(const aricc_vision_msgs::ObjectArray::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  const aricc_vision_msgs::Object& last_object = msg->objects.back();

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_color_optical_frame"; // Camera Link used
    transformStamped.child_frame_id = last_object.name; // Object name
    transformStamped.transform.translation.x -= last_object.position.y; // Object position x
    transformStamped.transform.translation.y -= last_object.position.x; // Object position y
    transformStamped.transform.translation.z = last_object.position.z; // Object position z
    

    tf2::Quaternion q;
    
    q.setRPY(1.57, -1.57, -last_object.orientation.z);
    transformStamped.transform.rotation.x = q.x() ;
    transformStamped.transform.rotation.y = q.y() ;
    transformStamped.transform.rotation.z = q.z() ;
    transformStamped.transform.rotation.w = q.w() ;
    br.sendTransform(transformStamped);

    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "object_tf2_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<aricc_vision_msgs::ObjectArray>("/re_dl_detection/YOLOv8RotateObjectDetection/dl_output", 1000, &poseCallback);
  ROS_INFO("Object_tf2_broadcaster node starting..");
  ros::Rate rate(10); // 10 Hz
  ros::spin();
  return 0;
};
