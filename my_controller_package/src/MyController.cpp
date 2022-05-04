#include <iostream>
#include <numeric>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include <loewen_msgs/Trajectory.h>
#include <loewen_msgs/DriveControl.h>
#include <utils/isfl_math.h>
#include <utils/isfl_geometry.h>
#include <utils/trajectory_utils.h>
#include <utils/isfl_ros_tf.h>
#include <common/CarState.h>
#include <common/ISteeringController.h>

#include "MyController.h"

//my
#include<cstring>
#include<stdlib.h>
#include<vector>
#include<math.h>
#include<cmath>
#include<CarState.h>

#include <ros/ros.h>


MyController::MyController(ros::NodeHandle &node_handle)
{
    // Set default config for all parameters.
    this->set_config(my_controller_package::MyControllerParameterConfig::__getDefault__());

    // Set node handle and publisher to enable debugging with markers in rviz using ROS.
    this->node_handle = node_handle;
    this->pub_marker = this->node_handle.advertise<visualization_msgs::Marker>("my_marker_topic", 1, true);
}

ControlCmd MyController::calculate_drive_control(
    const loewen_msgs::Trajectory &trajectory,
    const CarState &estimated_state,
    const TrajectoryLocation &car_loc,
    float time_delta)
{
    // #########################################################################
    // ####  Implement desired control algorithm to follow the trajectory.  ####
    // #########################################################################

    // TODO: Remove example 1: Show marker in rviz. Config parameters can be changed during runtime.
    double marker_x = 1.0;
    double marker_y = this->config.my_custom_parameter_1;
    this->show_marker(1, marker_x, marker_y, 1, 0, 0);

    // TODO: Remove example 2: Get estimated back axle position and show marker in rviz.
    double car_rear_x = estimated_state.rear_axle.x;
    double car_rear_y = estimated_state.rear_axle.y;
    this->show_marker(2, car_rear_x, car_rear_y, 0, 1, 0);

    // TODO: Remove example 3: Get closest trajectory point to a given position and show marker in rviz.
    TrajectoryLocation rear_loc = TrajectoryLocation(estimated_state.rear_axle, trajectory);
    loewen_msgs::TrajectoryPoint closest_point = trajectory.points[rear_loc.trajectory_idx];
    double traj_rear_x = closest_point.x;
    double traj_rear_y = closest_point.y;
    this->show_marker(3, traj_rear_x, traj_rear_y, 0, 0, 1);

    // TODO: Implement your control algorithm here.
    // pure suit

    using namespace std;
    
    int target_index = determine_closest_index(trajectory, estimated_state.rear_axle, 0, 1.5);
    loewen_msgs::TrajectoryPoint target_point = trajectory.points[target_index];
    
    double tx = target_point.x;
    double ty = target_point.y;
    
    double x = estimated_state.rear_axle.x ;
    double y = estimated_state.rear_axle.y ;

    double dt = 0.1; //decision time periodic
    
    double delta = 0; // front_angle
    double yaw   = 0 ; //Richtungswinkel
    
    double L = abs(sqrt(pow(estimated_state.rear_axle.x-estimated_state.front_axle.x,2)+pow(estimated_state.rear_axle.y-estimated_state.front_axle.y,2)));//zhou ju
    
    double Kp = 1.0;// speed control index
    double a  = Kp * abs(delta);//speed index
    double v  = 3- a*dt;
    
    double alpha = atan2(ty-y,tx-x)-yaw;
    
    double K   = 0.1;//look ahead index
    double Lfc = 1.5;//constant
    double Lf  = K*v + Lfc;//look ahead distance


    double abw = abs(ty-y); //Abweichung für y
    if ( abw > 0.02 )
    {
        delta = atan2(2.0*L*sin(alpha)/Lf, 1.0);
    }
    else
    {
        delta = 0.2*delta;
    }



    x   += estimated_state.speed_mps * cos(estimated_state.rear_angle) * dt;
    y   += estimated_state.speed_mps * sin(estimated_state.rear_angle) * dt;
    yaw  = estimated_state.speed_mps / L*tan(delta)*dt;

        

    // ##########################################################
    // ####  Set the control command to be sent to the car.  ####
    // ##########################################################

    // TODO: Set the control command.
    ControlCmd cmd{};
    cmd.speed_mps   = v;
    cmd.front_angle = delta;
    cmd.rear_angle  = 0.00;

    // Return command.
    return cmd;
}

void MyController::set_config(const my_controller_package::MyControllerParameterConfig &cfg)
{
    this->config = cfg;
}

void MyController::set_constraints(const control::ConstraintsConfig &c)
{
    this->constraints = c;
}

void MyController::reset()
{

}

void MyController::show_marker(int id, double x, double y, double r, double g, double b)
{
    // Create marker message to be published.
    visualization_msgs::Marker marker;
    // Set marker data.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
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
    this->pub_marker.publish(marker);
}