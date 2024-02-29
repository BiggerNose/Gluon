#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include "aricc_2d_vision/rotate_object_detection.h"
#include "aricc_utils/geometry_utils.h"


// Initialize global variables
double depth_value;
cv::Point2d last_position_;

void visionCallback(const aricc_vision_msgs::RotateObjectArray::ConstPtr& msg) {      
    if (!msg->items.empty()) {
        const aricc_vision_msgs::RotateObject & last_object = msg->items.back(); // get the last object in the message
        last_position_.x = last_object.center.x; // set the x-coordinate of the position to the object's x-coordinate
        last_position_.y = last_object.center.y; // set the y-coordinate of the position to the object's y-coordinate
    }        
    // ROS_INFO("Object at center: (%f : %f)", last_position_.x, last_position_.y);
}

void depthCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg) {
    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // Convert depth image to OpenCV format
    cv::Mat depth_image = cv_depth_ptr->image;

    if (last_position_.x >= 0 && last_position_.x < depth_image.cols && last_position_.y >= 0 && last_position_.y < depth_image.rows) {
        ROS_INFO("Detected");
        depth_value = depth_image.at<uint16_t>(last_position_.y, last_position_.x) * 0.001f;  // Convert depth value from mm to meters
        // ROS_INFO("Depth value at pixel (%f, %f): %.3f m", last_position_.x, last_position_.y, depth_value);
        ros::param::set("/Dynamic_Depth_Param", depth_value);
    }
    else {
        ROS_WARN("Pixel coordinates out of range");
    }
}

int main(int argc, char* argv[]) {
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "depth_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    ROS_INFO("Depth node starting");

    // Subscribe to the pixel image topic
    ros::Subscriber pixel_image_sub = nh.subscribe<aricc_vision_msgs::RotateObjectArray>("/jetson_rotate_yolov8/output/YOLOv8Rotate", 1, visionCallback);

    // Subscribe to the depth image topic
    ros::Subscriber depth_image_sub = nh.subscribe<sensor_msgs::Image>("Front_D435/depth/image_rect_raw", 1, depthCallback);

    // Don't exit the program.
    ros::spin();
}