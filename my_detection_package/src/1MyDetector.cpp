#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include <perception_utils/DetectorNodeHelper.h>
#include <perception_utils/DetectorInput.h>
#include <perception_utils/StreetMapWrapper.h>
#include <perception_utils/detection_utils.h>
#include <perception_utils/pixel_algorithms.h>

#include <visualization_msgs/Marker.h>
#include <loewen_msgs/ROI.h>
#include <loewen_msgs/StreetMap.h>
#include <loewen_msgs/ObjectClass.h>

#include <my_detection_package/MyDetectorParameterConfig.h>

#include "MyDetector.h"


MyDetector::MyDetector(ros::NodeHandle &node_handle)
{
    // BarredArea, Objects
    this->pub_blocked_detection = node_handle.advertise<loewen_msgs::ROI>("/segmentation_roi", 100);
    // crosswalk
    this->pub_crowsswalk_detection = node_handle.advertise<loewen_msgs::ROI>("/crosswalk_roi", 100);
    // segmentation_parking_spot_roi
    this->pub_parking_spot_detection = node_handle.advertise<loewen_msgs::ROI>("/segmentation_parking_spot_roi", 100);

    // Publisher to publish map markers.
    this->pub_map_marker = node_handle.advertise<visualization_msgs::Marker>("/my_marker_topic", 1, true);
}

void MyDetector::update(const DetectorInput &input)
{
    // Check if street is visable in current camera image.
    if(!input.get_street_map()->empty())
    {
        // TODO: Use your own detection algorithm.
        this->my_detection_algorithm(input);
        // this->example(input);
    }
}



void MyDetector::my_detection_algorithm(const DetectorInput &input)
{
    // ########################################################################################
    // ####  TODO: Remove example 1: Iterate over visible street in current camera image.  ####
    // ########################################################################################

    // Create StreetIterator on the right lane starting 0.10 meters in front of the car's DOM.
    StreetIterator iterator = input.get_street_map()->begin_right_at_car(0.10);
    float end_dom = input.get_street_map()->end_dom();

    int n[]={}; 
    int index = 0;

    // Iterate by incrementing the iterator (dom) value. Stepsize 0.05 meters in DOM.
    for(iterator; iterator < end_dom; iterator += 0.05)
    {
        // ##################################################################
        // ####  TODO: Remove example 2: Get street data from iterator.  ####
        // ##################################################################

        // Get street data from iterator.
        glm::vec2 current_point   = iterator.point;        // Point relative to car.
        glm::vec2 current_tangent = iterator.tangent;      // Tangent of street at that point.
        glm::vec2 current_normal  = iterator.normal;       // Left normal of street at that point.
        float current_lane_width  = iterator.lane_width;   // Width of street at that point.
        float current_dom         = iterator.current_distance_on_map();

        // Get value from reconfigurable parameters.
        float my_param = this->config.my_custom_parameter_1;
        float min_white_value = this->config.min_white_value;

        // #######################################################################
        // ####  TODO: Remove example 3: Get pixel in image of world points.  ####
        // #######################################################################

        // Get pixel in image of corresponding world point.
        cv::Point current_point_pixel = input.world_to_ipm(current_point);
        // Draw marker in debug image.
        this->draw_image_marker(input, iterator, current_point_pixel);
        // Get pixel value of current image. (Grayscale CV_8UC1 image. [0,255])
        int current_pixel_value = (int) input.ipm_raw_gray.at<uchar>(current_point_pixel);
        
        
        
        int start_dom = (int) round(input.get_street_map()->car_dom() + 3);

        if(current_pixel_value > min_white_value)
        {
            n[index]=current_dom;
            index++;

            if(n[0]==start_dom)
            {
                loewen_msgs::ROI obstacle_roi;
                // Set timestamp.
                obstacle_roi.header.stamp = input.timestamp;
                // Set location of detection.
                obstacle_roi.location.lane = loewen_msgs::StreetRefLocation::LANE_RIGHT;
                obstacle_roi.location.start_distance_on_map = start_dom ;
                obstacle_roi.location.length_on_map = 1.00;   // Length of detected object.
                // Set properties of detection.
                obstacle_roi.properties.object_class = loewen_msgs::ObjectClass::BarredArea;
                obstacle_roi.properties.offset_left  =  0.10; // 0.10 to left  from right lane center.
                obstacle_roi.properties.offset_right = -0.20; // 0.20 to right from right lane center.
                obstacle_roi.properties.width = obstacle_roi.properties.offset_left - obstacle_roi.properties.offset_right;
                // Publish detection. (Detections are only valid if detected in multiple iterations, otherweise they get ignored)
                this->pub_blocked_detection.publish(obstacle_roi);
                index = 0;
            } 
        }



        
        glm::vec2 left_lane_point = current_point + current_normal * current_lane_width ; // Explicit float needed
        // Get pixel value of that point.
        cv::Point left_point_pixel = input.world_to_ipm(left_lane_point);
        int leftlane_pixel_value = (int) input.ipm_raw_gray.at<uchar>(left_point_pixel);

    }

    // ######################################################################
    // ####  TODO: Remove example 4: Get world point of pixel in image.  ####
    // ######################################################################

    // // Define dummy pixel value.
    // cv::Point center_pixel = cv::Point(400, 300);
    // // Draw marker in debug image.
    // this->draw_image_marker(input, iterator, center_pixel, 0, 0, 255);
    // // Get world point of that pixel.
    // glm::vec2 center_point = input.ipm_to_world(center_pixel);
    // // Draw marker in map.
    // this->draw_map_marker(1, center_point.x, center_point.y, 0, 0, 1);

    // ############################################################################################
    // ####  TODO: Remove example 5: Publish random detection of barred areas and crosswalks.  ####
    // ############################################################################################

    // // Get dom 3 meters in front of car for random detection.
    // int random_dom = (int) round(input.get_street_map()->car_dom() + 3);

    // // Publish obstacle detection every 10 meters.
    // if(random_dom%10 == 5)
    // {
    //     // Create new obstacle msg to publish a detection.
    //     loewen_msgs::ROI obstacle_roi;
    //     // Set timestamp.
    //     obstacle_roi.header.stamp = input.timestamp;
    //     // Set location of detection.
    //     obstacle_roi.location.lane = loewen_msgs::StreetRefLocation::LANE_RIGHT;
    //     obstacle_roi.location.start_distance_on_map = random_dom;
    //     obstacle_roi.location.length_on_map = 1.00;   // Length of detected object.
    //     // Set properties of detection.
    //     obstacle_roi.properties.object_class = loewen_msgs::ObjectClass::BarredArea;
    //     obstacle_roi.properties.offset_left  =  0.10; // 0.10 to left  from right lane center.
    //     obstacle_roi.properties.offset_right = -0.20; // 0.20 to right from right lane center.
    //     obstacle_roi.properties.width = obstacle_roi.properties.offset_left - obstacle_roi.properties.offset_right;
    //     // Publish detection. (Detections are only valid if detected in multiple iterations, otherweise they get ignored)
    //     this->pub_blocked_detection.publish(obstacle_roi);
    // }

    // // Publish crosswalk detection every 30 meters.
    // if(random_dom%30 == 25)
    // {
    //     // Create new obstacle msg to publish a detection.
    //     loewen_msgs::ROI crosswalk_roi;
    //     // Set timestamp.
    //     crosswalk_roi.header.stamp = input.timestamp;
    //     // Set location of detection.
    //     crosswalk_roi.location.lane = loewen_msgs::StreetRefLocation::LANE_BOTH;
    //     crosswalk_roi.location.start_distance_on_map = random_dom;
    //     crosswalk_roi.location.length_on_map = 0.50;  // Length of detected crosswalk.
    //     // Set properties of detection.
    //     crosswalk_roi.properties.object_class = loewen_msgs::ObjectClass::Crosswalk;
    //     crosswalk_roi.properties.offset_left  =  0.0; // No offset for crosswalks.
    //     crosswalk_roi.properties.offset_right =  0.0; // No offset for crosswalks.
    //     crosswalk_roi.properties.width = crosswalk_roi.properties.offset_left - crosswalk_roi.properties.offset_right;
    //     crosswalk_roi.properties.wait_for_pedestrians = true; // Crosswalk is ignored when not set to true!
    //     // Publish detection. (Detections are only valid if detected in multiple iterations, otherweise they get ignored)
    //     this->pub_crowsswalk_detection.publish(crosswalk_roi);
    // }

}

void MyDetector::draw_image_marker(const DetectorInput &input, StreetIterator &iterator, cv::Point &point, int r, int g, int b, int radius)
{
    // Check if debug image exists. (Debug image is only created if there is a subscriber for it)
    if(input.get_debug_img() != nullptr)
    {
        // Get current debug image. (Dereferenced from pointer, shape=[600,800])
        cv::Mat debug_img = *input.get_debug_img();

        // Only draw in debug image. Do not change the input.ipm_raw_gray, which is for detection only.
        cv::circle(debug_img, point, radius, cv::Scalar(b,g,r), cv::FILLED);
    }
    else
    {
        ROS_WARN("Cannot display marker, because there is no subscriber on the debug image topic.");
    }
}

void MyDetector::draw_map_marker(int id, float x, float y, float r, float g, float b, float scale)
{
    // Create marker message to be published.
    visualization_msgs::Marker marker;
    // Set marker data.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.lifetime = ros::Duration(0.1);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    // Set marker pose.
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.00;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Publish marker to show it in rviz.
    this->pub_map_marker.publish(marker);
}

void MyDetector::config_callback(my_detection_package::MyDetectorParameterConfig &config, uint32_t level)
{
    this->config = config;
}
