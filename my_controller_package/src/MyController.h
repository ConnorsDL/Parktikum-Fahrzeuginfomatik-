#ifndef MY_CAR_CONTROLLER_H
#define MY_CAR_CONTROLLER_H

#include <glm/glm.hpp>
#include <deque>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <loewen_msgs/DriveControl.h>
#include <common/IDriveController.h>
#include <common/ISteeringController.h>
#include <utils/trajectory_utils.h>

#include <my_controller_package/MyControllerParameterConfig.h>


class MyController : public IDriveController
{
public:
    explicit MyController(ros::NodeHandle &node_handle);

    ControlCmd calculate_drive_control(
        const loewen_msgs::Trajectory &trajectory,
        const CarState &estimated_state,
        const TrajectoryLocation &estimated_trajectory_loc,
        float time_delta) override;

    void set_config(const my_controller_package::MyControllerParameterConfig &config);

    void set_constraints(const control::ConstraintsConfig &constraints) override;

    void reset() override;

private:
    // Debug methods to show markers in rviz.
    void show_marker(int id, double x, double y, double r=1.0, double g=0.0, double b=0.0);

    my_controller_package::MyControllerParameterConfig config;
    control::ConstraintsConfig constraints;

    ros::NodeHandle node_handle;
    ros::Publisher pub_marker;
};


#endif // MY_CAR_CONTROLLER_H
