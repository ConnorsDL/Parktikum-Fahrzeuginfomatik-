#pragma once

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


class MyDetector
{
public:
    MyDetector(ros::NodeHandle &node_handle);
    void update(const DetectorInput &input);
    void my_detection_algorithm(const DetectorInput &input);
    void example(const DetectorInput &input);
    void draw_image_marker(const DetectorInput &input, StreetIterator &iterator, cv::Point &point, int r=255, int g=0, int b=0, int radius=2);
    void draw_map_marker(int id, float x, float y, float r=1.0, float g=0.0, float b=0.0, float scale=0.1);
    void config_callback(my_detection_package::MyDetectorParameterConfig &config, uint32_t level);

private:
    ros::Publisher pub_blocked_detection;
    ros::Publisher pub_crowsswalk_detection;
    ros::Publisher pub_parking_spot_detection;

    ros::Publisher pub_map_marker;

    my_detection_package::MyDetectorParameterConfig config = my_detection_package::MyDetectorParameterConfig::__getDefault__();
};

