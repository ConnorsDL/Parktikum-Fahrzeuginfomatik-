cmake_minimum_required(VERSION 2.8.3)
project(my_obstacle_avoidance_package)

find_package(catkin REQUIRED COMPONENTS
    rospy
    dynamic_reconfigure
    std_msgs
    loewen_msgs
    waypoint_planning
)

# catkin_python_setup()

# generate_dynamic_reconfigure_options(
#     cfg/General.cfg
#     cfg/CircuitMode.cfg
#     cfg/ObstacleMode.cfg
#     cfg/Scheduling.cfg
# )

catkin_package(
    CATKIN_DEPENDS loewen_msgs utils loewen_msgs tf_conversions waypoint_planning
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
