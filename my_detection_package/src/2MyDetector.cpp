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
    if(!input.get_street_map()->empty())   //if not empty, go to algor...
    {
        // TODO: Use your own detection algorithm.
        this->my_detection_algorithm(input);
        //this->example(input);
    }
}

void MyDetector::my_detection_algorithm(const DetectorInput &input)
{
    // ###############################
    // ####  Detection Algorithm  ####
    // ###############################

    // TODO: Implement your detection algorithm here!
    // ...

//1: Iterate over visible street in current camera image. 
    // Create StreetIterator on the right lane starting 0.40 meters in front of the car's DOM.
    StreetIterator iterator = input.get_street_map()->begin_right_at_car(0.4);
    float end_dom = input.get_street_map()->end_dom();
    int block[]={}; 
    int block_index = 0;
    int crosswalk[]={};
    int crosswalk_index=0;
    // Iterate by incrementing the iterator (dom) value. Stepsize 0.05 meters in DOM.
    for(iterator; iterator < end_dom; iterator += 0.04)
    {
    //2: Get street data from iterator.
        glm::vec2 current_point   = iterator.point;        // Point relative to car.
        glm::vec2 current_tangent = iterator.tangent;      // Tangent of street at that point.
        glm::vec2 current_normal  = iterator.normal;       // Left normal of street at that point.
        float current_lane_width  = iterator.lane_width;   // Width of street at that point.
        float current_dom         = iterator.current_distance_on_map();

        // Get value from reconfigurable parameters.
        float my_param = this->config.my_custom_parameter_1;
        float min_white_value = this->config.min_white_value;


    //3: Get pixel in image of world points.
        // Get pixel in image of corresponding world point.
        cv::Point current_point_pixel = input.world_to_ipm(current_point);
        // Draw marker in debug image.
        this->draw_image_marker(input, iterator, current_point_pixel);
        // Get pixel value of current image. (Grayscale CV_8UC1 image. [0,255])
        int current_pixel_value = (int) input.ipm_raw_gray.at<uchar>(current_point_pixel);
        // std::cout << current_pixel_value << std::endl ;
        // std::cout << current_dom << std::endl ;
        
        //////lane1 in the left#################################################################
        glm::vec2 left_lane_point = current_point + current_normal * current_lane_width; // Explicit float needed
        // Get pixel value of that point.
        cv::Point left_point_pixel = input.world_to_ipm(left_lane_point);
        int leftlane_pixel_value = (int) input.ipm_raw_gray.at<uchar>(left_point_pixel);
        this->draw_image_marker(input, iterator, left_point_pixel, 0,0,255);

        //////lane2 in the left#################################################################
        glm::vec2 left_lane_point2 = current_point + current_normal * current_lane_width*1.15f ; // Explicit float needed
        // Get pixel value of that point.
        cv::Point left_point2_pixel = input.world_to_ipm(left_lane_point2);
        int leftlane2_pixel_value = (int) input.ipm_raw_gray.at<uchar>(left_point2_pixel);
        this->draw_image_marker(input, iterator, left_point2_pixel, 0,0,255);

        //////lane3 in the left#################################################################
        glm::vec2 left_lane_point3 = current_point + current_normal * current_lane_width*0.85f; // Explicit float needed
        // Get pixel value of that point.
        cv::Point left_point3_pixel = input.world_to_ipm(left_lane_point3);
        int leftlane3_pixel_value = (int) input.ipm_raw_gray.at<uchar>(left_point3_pixel);
        this->draw_image_marker(input, iterator, left_point3_pixel, 0,0,255);

        //////lane4 in the left#################################################################
        glm::vec2 left_lane_point4 = current_point + current_normal * current_lane_width*0.7f; // Explicit float needed
        // Get pixel value of that point.
        cv::Point left_point4_pixel = input.world_to_ipm(left_lane_point4);
        int leftlane4_pixel_value = (int) input.ipm_raw_gray.at<uchar>(left_point4_pixel);
        this->draw_image_marker(input, iterator, left_point4_pixel, 0,0,255);


        //////lane2 in the right#################################################################
         glm::vec2 right_lane_point2 = current_point + current_normal * current_lane_width/4.0f ; // Explicit float needed
        // Get pixel value of that point.
        cv::Point right_point2_pixel = input.world_to_ipm(right_lane_point2);
        int rightlane2_pixel_value = (int) input.ipm_raw_gray.at<uchar>(right_point2_pixel);
        this->draw_image_marker(input, iterator, right_point2_pixel, 255,255,0);

         //////lane3 in the right#################################################################
         glm::vec2 right_lane_point3 = current_point - current_normal * current_lane_width/3.0f ; // Explicit float needed
        // Get pixel value of that point.
        cv::Point right_point3_pixel = input.world_to_ipm(right_lane_point3);
        int rightlane3_pixel_value = (int) input.ipm_raw_gray.at<uchar>(right_point3_pixel);
        this->draw_image_marker(input, iterator, right_point3_pixel, 255,255,0);

         //////lane4 in the right#################################################################
         glm::vec2 right_lane_point4 = current_point - current_normal * current_lane_width/3.5f ; // Explicit float needed
        // Get pixel value of that point.
        cv::Point right_point4_pixel = input.world_to_ipm(right_lane_point4);
        int rightlane4_pixel_value = (int) input.ipm_raw_gray.at<uchar>(right_point4_pixel);
        this->draw_image_marker(input, iterator, right_point4_pixel, 255,255,0);



        //int random_dom = (int) round(input.get_street_map()->car_dom() + 1);
        //int car_dom = (int) round(input.get_street_map()->car_dom());
        int start_dom = (int) round(input.get_street_map()->car_dom() + 1);
         float ABS;
         float ABS2;
        // int l = current_dom - car_dom;
        //std::cout << car_dom << std::endl ;
        
        int i1=leftlane_pixel_value> min_white_value;
        int i2=leftlane2_pixel_value> min_white_value;
        int i3=leftlane3_pixel_value> min_white_value;
        int i4=leftlane4_pixel_value> min_white_value;
        int j1=current_pixel_value> min_white_value;
        int j2=rightlane2_pixel_value> min_white_value;
        int j3=rightlane3_pixel_value> min_white_value;
        int j4=rightlane4_pixel_value> min_white_value;
        int i=i1+i2+i3+i4;
        int j=j1+j2+j3+j4;
        std::cout << i << std::endl ;
       
        //if ((current_pixel_value > min_white_value)&& (leftlane_pixel_value < min_white_value)&&(leftlane2_pixel_value < min_white_value) )
        if(i<2&&j>=1)
        {
            block[block_index]=current_dom;
            block_index++; 
            
            if(block[0]>=start_dom)
            {
                ABS=block[0]-start_dom; 
            }
            else
            {
                ABS=start_dom-block[0];
            }
            if(ABS<0.5)
            {
                loewen_msgs::ROI obstacle_roi;
                // Set timestamp.
                obstacle_roi.header.stamp = input.timestamp;
                // Set location of detection.
                obstacle_roi.location.lane = loewen_msgs::StreetRefLocation::LANE_RIGHT;
                obstacle_roi.location.start_distance_on_map = start_dom -0.5;
                obstacle_roi.location.length_on_map = 1.10;   // Length of detected object.
                // Set properties of detection.
                obstacle_roi.properties.object_class = loewen_msgs::ObjectClass::BarredArea;
                obstacle_roi.properties.offset_left  =  0.10; // 0.10 to left  from right lane center.
                obstacle_roi.properties.offset_right = -0.20; // 0.20 to right from right lane center.
                obstacle_roi.properties.width = obstacle_roi.properties.offset_left - obstacle_roi.properties.offset_right;
                // Publish detection. (Detections are only valid if detected in multiple iterations, otherweise they get ignored)
                this->pub_blocked_detection.publish(obstacle_roi);
                block_index = 0;
                
            }   
       
            
            
            // if(leftlane_pixel_value>min_white_value)
            // {
            //     crosswalk_counter++;
            // }
            // else
            // {
            //     block_counter++;
            // }
            
        }
        //else if ((leftlane_pixel_value > min_white_value)||(leftlane2_pixel_value > min_white_value))
        else if(i>=2&&j>=1)
        {
            crosswalk[crosswalk_index]=current_dom;
            crosswalk_index++; 
            
            if(crosswalk[0]>=start_dom)
            {
                ABS2=crosswalk[0]-start_dom; 
            }
            else
            {
                ABS2=start_dom-crosswalk[0];
            }
            if(ABS2<0.3)
            {
                // Create new obstacle msg to publish a detection.
                loewen_msgs::ROI crosswalk_roi;
                // Set timestamp.
                crosswalk_roi.header.stamp = input.timestamp;
                // Set location of detection.
                crosswalk_roi.location.lane = loewen_msgs::StreetRefLocation::LANE_BOTH;
                crosswalk_roi.location.start_distance_on_map = start_dom-0.5;
                crosswalk_roi.location.length_on_map = 0.50;  // Length of detected crosswalk.
                // Set properties of detection.
                crosswalk_roi.properties.object_class = loewen_msgs::ObjectClass::Crosswalk;
                crosswalk_roi.properties.offset_left  =  0.0; // No offset for crosswalks.
                crosswalk_roi.properties.offset_right =  0.0; // No offset for crosswalks.
                crosswalk_roi.properties.width = crosswalk_roi.properties.offset_left - crosswalk_roi.properties.offset_right;
                crosswalk_roi.properties.wait_for_pedestrians = true; // Crosswalk is ignored when not set to true!
                // Publish detection. (Detections are only valid if detected in multiple iterations, otherweise they get ignored)
                this->pub_crowsswalk_detection.publish(crosswalk_roi);
                crosswalk_index=0;
                
            }
        }
        
        
        // Compute custom world point.
        // glm::vec2 side_point = current_point - current_normal * current_lane_width / 2.0f; // Explicit float needed
        // // Get pixel value of that point.
        // cv::Point side_point_pixel = input.world_to_ipm(side_point);
        // // Draw marker in debug image.
        // this->draw_image_marker(input, iterator, side_point_pixel, 0, 255, 0);

        // find_closest_white_pixel_transverse( cv::Point , current_point,
        //                             const glm::vec2 &direction, 2.0f,
        //                             0, input.world_to_ipm);


    }






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
