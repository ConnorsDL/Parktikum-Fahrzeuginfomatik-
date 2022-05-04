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


int main(int argc, char **argv)
{
    // Init ROS node.
    ros::init(argc, argv, "my_detector_node");
    ros::NodeHandle node_handle;
    MyDetector my_detector(node_handle);
    dynamic_reconfigure::Server<my_detection_package::MyDetectorParameterConfig> server;
    dynamic_reconfigure::Server<my_detection_package::MyDetectorParameterConfig>::CallbackType f;
    f = boost::bind(&MyDetector::config_callback, &my_detector, _1, _2);
    server.setCallback(f);
    ros::Rate rate(30);
    // DetectorNodeHelper helper{node_handle, {"ipm/image_raw"}};
    DetectorNodeHelper helper{node_handle, {"ipm/image_raw_white_mask"}};
    helper.require_odometry();
    helper.require_street_map();
    helper.set_debug_topic("ipm/my_detector_node_debug");
    helper.set_callback([&](const DetectorInput &input)
    {
        my_detector.update(input);
        rate.sleep();
    });
    ros::spin();
}
