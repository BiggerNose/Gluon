#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "apriltag_ros/AprilTagDetection.h"
// #include "apriltag_ros/AprilTagDetectionArray.h"
#include "aricc_vision_msgs/ObjectArray.h"
#include "aricc_vision_msgs/Object.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <myworkcell_core/LocalizePart.h>

class Localizer
{
public:
    Localizer(ros::NodeHandle& nh) : listener_(buffer_)
    {
        // Subscribe to object poses
        object_pose_sub = nh.subscribe<aricc_vision_msgs::ObjectArray>(
            "/re_dl_detection/YOLOv8RotateObjectDetection/dl_output", 1000, 
            &Localizer::objectCallback, this);

        // Advertise the localize_part service
        server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
    }

    void objectCallback(const aricc_vision_msgs::ObjectArray::ConstPtr& msg)
{
    if (!msg->objects.empty())
    {
        // Iterate through detected objects
        for (const aricc_vision_msgs::Object& object : msg->objects)
        {
            if (object.name == "S40_40_B")
            {
        
                object_name = object.name;

                // Assuming the orientation is already in the correct format
                myQuaternion.setRPY(object_pose.pose.orientation.x,object_pose.pose.orientation.y,object_pose.pose.orientation.z);
                tf2::convert(myQuaternion, object_pose.pose.orientation);
                return;  // Exit the loop after finding the specific object
            }
        }
    }
}

    bool localizePart(myworkcell_core::LocalizePart::Request& req,
                      myworkcell_core::LocalizePart::Response& res)
    {    
        geometry_msgs::PoseStamped target_pose_from_cam;
        target_pose_from_cam.header.frame_id = object_name;
       ROS_INFO_STREAM("object pose:" << object_pose ) ; 

        target_pose_from_cam.pose = object_pose.pose;

       ROS_INFO("Transformed pose: x=%f, y=%f, z=%f",  target_pose_from_cam.pose.position.x,  target_pose_from_cam.pose.position.y,  target_pose_from_cam.pose.position.z);

        // Transform the pose to the target frame
        try
        {   
            res.pose = buffer_.transform(target_pose_from_cam, req.base_frame).pose;

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Failed to transform pose: %s", ex.what());
            return false;
        }
    }

private:
    ros::Subscriber object_pose_sub;
    geometry_msgs::PoseStamped object_pose;
    aricc_vision_msgs::Object last_object;
    std::string object_name;
    tf2::Quaternion myQuaternion;
    tf2::Quaternion myQuaternion1;
    ros::ServiceServer server_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    Localizer localizer(nh);

    ROS_INFO("Vision node starting");
    
    ros::Rate rate(10); // Adjust the rate as needed

    // Don't exit the program.
    ros::spin();

    return 0;
}


