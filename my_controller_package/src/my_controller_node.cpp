#include <memory>
#include <ros/init.h>
#include <dynamic_reconfigure/server.h>

#include <common/ControlNode.h>
#include <my_controller_package/MyControllerParameterConfig.h>

#include "MyController.h"

int main(int argc, char **argv)
{
    // Init ROS node.
    ros::init(argc, argv, "my_car_controller_node");

    // Init ROS node handle for ROS communications.
    ros::NodeHandle node_handle;

    // Create the controller to control the car.
    auto controller = std::make_shared<MyController>(node_handle);

    // Start the dynamic reconfiguration server to change parameters during runtime.
    auto server = std::make_shared<dynamic_reconfigure::Server<my_controller_package::MyControllerParameterConfig>>();
    server->setCallback([controller](my_controller_package::MyControllerParameterConfig &config, int level) -> void {
        controller->set_config(config);
    });

    // Start the controller.
    ControlNode node(node_handle, controller);

    // Spin for ROS subscriber callbacks the be called until the node is shutdown.
    ros::spin();
}
