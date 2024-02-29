#include "pick_and_place_control/pick_and_place.h"

 
PickAndPlace::PickAndPlace(ros::NodeHandle& nh) : node_handle(nh), listener_(buffer_), move_group("gluon_arm") { 

    pixel_image_sub = nh.subscribe<aricc_vision_msgs::RotateObjectArray>(
        "/jetson_rotate_yolov8/output/YOLOv8Rotate", 1, &PickAndPlace::visionCallback, this); 
    depth_image_sub = nh.subscribe<sensor_msgs::Image>(
        "Front_D435/depth/image_rect_raw", 1, &PickAndPlace::depthCallback, this); 
    object_sub = nh.subscribe<aricc_vision_msgs::ObjectArray>(
        "/re_dl_detection/YOLOv8RotateObjectDetection/dl_output", 1, &PickAndPlace::objectCallback, this);        
}


void PickAndPlace::objectCallback(const aricc_vision_msgs::ObjectArray::ConstPtr& msg) {
        // Create a transform broadcaster
        ROS_INFO("im objectcallback");
        geometry_msgs::TransformStamped transformStamped;
        static tf2_ros::TransformBroadcaster tf_broadcaster;
        last_object_ = msg->objects.back();
        // Create and broadcast a transform
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "camera_color_optical_frame"; // Parent frame
        transformStamped.child_frame_id = last_object_.name; // Child frame

        transformStamped.transform.translation.x -= last_object_.position.y; // Object position x
        transformStamped.transform.translation.y -= last_object_.position.x; // Object position y
        transformStamped.transform.translation.z = last_object_.position.z;

        tf2::Quaternion q;
        q.setRPY(1.57, -1.57, -last_object_.orientation.z);
        
        transformStamped.transform.rotation.x = q.x(); // Rotation
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        tf_broadcaster.sendTransform(transformStamped);
        // Set the static transform flag to true
        ROS_INFO_STREAM(transformStamped);
        
}




void PickAndPlace::visionCallback(const aricc_vision_msgs::RotateObjectArray::ConstPtr& msg) {   
            ROS_INFO("im Vsioncallback");
  
    if (!msg->items.empty()) {
        const aricc_vision_msgs::RotateObject & last_object = msg->items.back(); // get the last object in the message
        last_position_.x = last_object.center.x; // set the x-coordinate of the position to the object's x-coordinate
        last_position_.y = last_object.center.y; // set the y-coordinate of the position to the object's y-coordinate
        }   
     
}




void PickAndPlace::depthCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg) {

    ROS_INFO("im depthcallback");

    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // Convert depth image to OpenCV format
    cv::Mat depth_image = cv_depth_ptr->image;

    if (last_position_.x >= 0 && last_position_.x < depth_image.cols && last_position_.y >= 0 && last_position_.y < depth_image.rows) {
        depth_value = depth_image.at<uint16_t>(last_position_.y, last_position_.x) * 0.001f;  // Convert depth value from mm to meters
        // ROS_INFO("Depth value at pixel (%f, %f): %.3f m", last_position_.x, last_position_.y, depth_value);
        ros::param::set("/Dynamic_Depth_Param", depth_value);
    }
    else {
        ROS_WARN("Pixel coordinates out of range");
    }
}



bool PickAndPlace::transformation()
{   
    ROS_INFO("im transformation");

    ROS_INFO_STREAM(last_object_.name);
    geometry_msgs::TransformStamped transformStamped1;
    // geometry_msgs::Pose move_target; 
    try {
        transformStamped1 = buffer_.lookupTransform("base_link",last_object_.name,ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed to lookup transform: %s", ex.what());
    }
    move_target.position.x = transformStamped1.transform.translation.x;
    move_target.position.y = transformStamped1.transform.translation.y;
    move_target.position.z = transformStamped1.transform.translation.z;
    move_target.orientation.x = transformStamped1.transform.rotation.x ;
    move_target.orientation.y = transformStamped1.transform.rotation.y;
    move_target.orientation.z = transformStamped1.transform.rotation.z;
    move_target.orientation.w = transformStamped1.transform.rotation.w;
    move_target.position.z = 0.3 ;
    return true; 
 }



bool PickAndPlace::moveit()
{   
    ROS_INFO("MOVING IT");
    // ROS_INFO_STREAM(move_target);
    move_group.setPlanningTime(10);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setPoseTarget(move_target);
    ROS_INFO_STREAM(move_target);
    move_group.move();
    ROS_INFO("i have finish movinig");
    return true ;
}

bool PickAndPlace::lookup()
{   
    ROS_INFO("MOVING IT");
    // ROS_INFO_STREAM(move_target);
    move_group.setPlanningTime(10);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setNamedTarget("look_table_pose1");
    // move_goup.setPoseTarget(move_target);
    ROS_INFO_STREAM(move_target);
    move_group.move();
    ROS_INFO("i have finish movinig");
    return true ;
}




