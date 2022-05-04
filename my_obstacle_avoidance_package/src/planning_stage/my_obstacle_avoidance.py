#!/usr/bin/env python
from distutils.command.build_scripts import first_line_re
import rospy
import numpy as np
from typing import List
from sympy import Symbol, solve

import waypoint_planning.config as config
from waypoint_planning.Plan import Plan

from utils.map.MapWrapper import MapWrapper, MapLocation
from utils.geometry import distance, vector_of, angle_of, left_orthogonal, right_orthogonal, normalize
from utils.geometry.street import gen_curve_points_2d, gen_straight_points_2d, gen_straight_points_p2p

from loewen_msgs.msg import VerifiedObject, StreetRefLocation, ObjectProperties
from visualization_msgs.msg import Marker


# Helper for debugging. (Publishers are usually defined within a class)
marker_pub = None

###################################
####  TODO: splinder funktion  ####
###################################


def compute_spline(t0,f0,t1,f1,t2,f2,t3,f3):

#f_0 = 1at³ + 1bt² + 1ct + 1d
#f_1 = 3at² + 2bt  + 1c  + 0d

    A = np.array([
        [ 1*t0**3 , 1*t0**2 , 1*t0 , 1 ],
        [ 1*t1**3 , 1*t1**2 , 1*t1 , 1 ],
        [ 3*t2**2 , 2*t2 , 1 , 0 ],
        [ 3*t3**2 , 2*t3 , 1 , 0 ]
    ], dtype="float32")

    f = np.array([
        [f0],
        [f1],
        [f2],
        [f3]
    ], dtype="float32")

    x = np.linalg.solve(A, f)

    a = x[0,0]
    b = x[1,0]
    c = x[2,0]
    d = x[3,0]

    return a,b,c,d
    

    
   


def plan_my_obstacle_avoidance(plan: Plan, map_w: MapWrapper):
    # type: (Plan, MapWrapper) -> None
    """
    Modify the points in the plan so that the detected obstacles and barred areas are avoided.
    :param plan: the driving plan to be modified
    :param map_w: map wrapper
    """

    ##################################
    ####  Get obstacles from map  ####
    ##################################

    # Get obstacles from map.
    obstacles = map_w.obstacles + map_w.barred_areas
    # Filter out objects not on street.
    obstacles = [
        obstacle for obstacle in obstacles if map_w.is_on_main_street(obstacle.id)]
    # Sort objects by start_distance_on_map.
    obstacles.sort(
        key=lambda obstacle: obstacle.location.start_distance_on_map)

    ########################################
    ####  Obstacle Avoidance Algorithm  ####
    ########################################

    # TODO: Implement your obstacle avoidance here!
    # The current plan just follows the left lane and you have to change it to avoid obstacles by
    # moving out of the left lane before an obstacle and moving back into the left lane after the obstacle.
    # Keep in mind that you also have to consider obstacles before and after an obstacle when changing lane.
    # To compute your waypoints you can use functions provided in ros\utils\src\utils\geometry\street.py or
    # implement your own geometric calculation functions (spline interpolation, sigmoid parametrisation, ...)

    # example(plan, map_w, obstacles)
    # # ...

    my_test(plan, map_w, obstacles)


def my_test(plan: Plan, map_w: MapWrapper, obstacles: List[VerifiedObject]):
    # Show marker at map origin.
    show_marker(id=0, x=0, y=0, r=1, g=0, b=0, scale=0.1, a=1.0)

    ##Get car information.##
    car_width = config.general.car_width
    car_length = config.general.car_length

    # j = len(obstacles)
    for j in range(len(obstacles)):

        # if len(obstacles) == 1:
        ##Get obstacle information.##
        obstacle: VerifiedObject = obstacles[j]
        obstacle_street_loc: StreetRefLocation = obstacle.location
        obstacle_properties: ObjectProperties = obstacle.properties

        # Get obstacle data.
        # Distance on map where the obstacle starts.
        obstacle_start_dom = obstacle_street_loc.start_distance_on_map
        # Length of the obstacle.
        obstacle_length = obstacle_street_loc.length_on_map
        # Lane that the obstacle is on.
        obstacle_lane = obstacle_street_loc.lane
        # Positive distance between obstacle left side and the lane center it is on.
        obstacle_offset_left = obstacle_properties.offset_left
        # Negative distance between obstacle right side and the lane center it is on.
        obstacle_offset_right = obstacle_properties.offset_right
        # Width of the object.
        obstacle_width = obstacle_properties.width

    ##Get data from map.##

        # Get MapLocation by distance_on_map.
        obstacle_map_loc: MapLocation = map_w.locate_distance_on_map(
            obstacle_start_dom)

        # Get left/right street points and tangent from MapLocation.
        left_point = obstacle_map_loc.left_lane_point
        right_point = obstacle_map_loc.right_lane_point
        map_tangent = obstacle_map_loc.street_tangent
        map_angle = angle_of(map_tangent)

        # Get front, left, right and back vectors relative to map.
        map_front = normalize(map_tangent)
        map_left = normalize(left_orthogonal(map_front))
        map_right = map_left * -1.0
        map_back = map_front * -1.0
        # if len(obstacles) == 1:
        # Check if obstacle is on left or right lane.
        if obstacle_lane == StreetRefLocation.LANE_LEFT:
            show_marker(1,  left_point[0],  left_point[1], 0, 1, 0)
        else:
            show_marker(1, right_point[0], right_point[1], 0, 0, 1)

    ## Get data from current plan.##

        # Define distance_on_map where to access the current plan.
        my_dom = obstacle_start_dom - 0.75

        # Get point and tangent by distance_on_map in current plan.
        idx = plan.determine_idx_before_distance_on_map(my_dom)
        plan_point = plan.points[idx]
        plan_tanget = plan.tangents[idx]
        plan_angle = angle_of(plan_tanget)

      

        # Get front, left, right and back vectors relative to current plan.
        plan_front = normalize(plan_tanget)
        plan_left = normalize(left_orthogonal(plan_front))
        plan_right = plan_left * -1.0
        plan_back = plan_front * -1.0

        # Show marker for point in curmy_domrent plan.
        show_marker(2, plan_point[0], plan_point[1], 0, 1, 1)

        # end_dom = obstacle_start_dom + obstacle_length + 0.5

        # ##from left to right##
        # end_point1 = right_point + plan_front*obstacle_length/2
        # end_point1_tangent = map_tangent
        # tangen_y = plan_angle
        # my_points_1 = Curve_generator(
        #     plan_point[0], plan_point[1], tangen_y, end_point1[0], end_point1[1], tangen_y)

        # ##from right to left##
        # end_point3 = left_point + plan_front*(obstacle_length + 0.75)
        # my_points_3 = Curve_generator(
        #     end_point1[0], end_point1[1], tangen_y, end_point3[0], end_point3[1], tangen_y)

        end_point1 = right_point + plan_front*obstacle_length/2
        end_point1_tangent = map_tangent





        if obstacle_lane == StreetRefLocation.LANE_LEFT:

            if (j > 0):

                # get obstacle j-1 information
                obstacle_before: VerifiedObject = obstacles[j-1]
                obstacle_before_street_loc: StreetRefLocation = obstacle_before.location
                obstacle_before_properties: ObjectProperties = obstacle_before.properties
                obstacle_before_start_dom = obstacle_before_street_loc.start_distance_on_map
                obstacle_before_length = obstacle_before_street_loc.length_on_map
                obstacle_before_lane = obstacle_before_street_loc.lane

                # Get MapLocation by distance_on_map.
                obstacle_map_loc: MapLocation = map_w.locate_distance_on_map(
                    obstacle_before_start_dom)

                # Get left/right street points and tangent from MapLocation.
                before_left_point = obstacle_map_loc.left_lane_point
                before_right_point = obstacle_map_loc.right_lane_point

            if j == 0:
                end_point1 = right_point + plan_front*obstacle_length/2
                end_point1_tangent = map_tangent

                ##from left to right##
                #compute spline for x
                t0, f0 =0, plan_point[0]
                t1, f1 =1, end_point1[0]
                t2, f2 =0, plan_tanget[0]
                t3, f3 =1, end_point1_tangent[0]
                a0, b0, c0, d0= compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                #compute spline for y
                t0, f0 =0, plan_point[1]
                t1, f1 =1, end_point1[1]
                t2, f2 =0, plan_tanget[1]
                t3, f3 =1, end_point1_tangent[1]
                a1, b1, c1, d1 = compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                #compute x and y
                t= np.linspace(0,1,10)
                x=a0*t**3 + b0*t**2 + c0*t + d0
                y=a1*t**3 + b1*t**2 + c1*t + d1

                my_points_1 = np.array([x,y],dtype="float32").T

                my_tolerance = np.ones(
                    my_points_1.shape[0], dtype="float32") * 0.05
                # Set critical tolerance (default: False). This is only noticable on hardware.
                my_critical_tolerance = np.ones(
                    my_points_1.shape[0], dtype="bool")

                start_dom = my_dom

                end_dom = my_dom + 0.75 + obstacle_length / 2
                plan.replace_points_by_dist_on_map(
                    start_dom, end_dom, my_points_1,
                    left_tolerance=my_tolerance,
                    left_tolerance_critical=my_critical_tolerance
                )

                # Plan was changed now.
                # To visualize the difference between map and plan data we get another point from both.
                # 0.50 meters in front of where we started to change the plan.
                dom = my_dom + 0.75

                # Get map point at that distance_on_map.
                map_loc: MapLocation = map_w.locate_distance_on_map(dom)
                left_map_point = map_loc.left_lane_point

                # Get plan point at that distance_on_map.
                idx = plan.determine_idx_before_distance_on_map(dom)
                plan_point = plan.points[idx]

                # Show markers.
                show_marker(2, left_map_point[0], left_map_point[1], 0, 1, 0)
                show_marker(3, plan_point[0], plan_point[1], 0, 1, 1)

            elif(j > 0 and obstacle_before_lane == StreetRefLocation.LANE_RIGHT):
                ##from left to right##
                #compute spline for x
                end_point1 = right_point + plan_front*obstacle_length/2
                end_point1_tangent = map_tangent

                t0, f0 =0, plan_point[0]
                t1, f1 =1, end_point1[0]
                t2, f2 =0, plan_tanget[0]
                t3, f3 =1, end_point1_tangent[0]
                a0, b0, c0, d0= compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                #compute spline for y
                t0, f0 =0, plan_point[1]
                t1, f1 =1, end_point1[1]
                t2, f2 =0, plan_tanget[1]
                t3, f3 =1, end_point1_tangent[1]
                a1, b1, c1, d1 = compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                #compute x and y
                t= np.linspace(0,1,10)
                x=a0*t**3 + b0*t**2 + c0*t + d0
                y=a1*t**3 + b1*t**2 + c1*t + d1

                my_points_1 = np.array([x,y],dtype="float32").T

                my_tolerance = np.ones(
                    my_points_1.shape[0], dtype="float32") * 0.05
                # Set critical tolerance (default: False). This is only noticable on hardware.
                my_critical_tolerance = np.ones(
                    my_points_1.shape[0], dtype="bool")

                start_dom = my_dom

                end_dom = my_dom + 0.75 + obstacle_length / 2
                plan.replace_points_by_dist_on_map(
                    start_dom, end_dom, my_points_1,
                    left_tolerance=my_tolerance,
                    left_tolerance_critical=my_critical_tolerance
                )

                # Plan was changed now.
                # To visualize the difference between map and plan data we get another point from both.
                # 0.50 meters in front of where we started to change the plan.
                dom = my_dom + 0.75

                # Get map point at that distance_on_map.
                map_loc: MapLocation = map_w.locate_distance_on_map(dom)
                left_map_point = map_loc.left_lane_point

                # Get plan point at that distance_on_map.
                idx = plan.determine_idx_before_distance_on_map(dom)
                plan_point = plan.points[idx]

                # Show markers.
                show_marker(2, left_map_point[0], left_map_point[1], 0, 1, 0)
                show_marker(3, plan_point[0], plan_point[1], 0, 1, 1)

            if j < len(obstacles) - 1:
                ##Get obstacle j+1 information.##
                obstacle_next: VerifiedObject = obstacles[j+1]
                obstacle_next_street_loc: StreetRefLocation = obstacle_next.location
                obstacle_next_properties: ObjectProperties = obstacle_next.properties
                obstacle_next_start_dom = obstacle_next_street_loc.start_distance_on_map
                obstacle_next_length = obstacle_next_street_loc.length_on_map
                obstacle_next_lane = obstacle_next_street_loc.lane

                ##Get data from map.##

                # Get MapLocation by distance_on_map.
                obstacle_map_loc: MapLocation = map_w.locate_distance_on_map(
                    obstacle_next_start_dom)

                # Get left/right street points and tangent from MapLocation.
                next_left_point = obstacle_map_loc.left_lane_point
                next_right_point = obstacle_map_loc.right_lane_point

                if (obstacle_next_lane == StreetRefLocation.LANE_LEFT and
                        (distance(next_left_point, left_point) - obstacle_length) < 4*car_length):

                    my_points_2_end_point = next_right_point + plan_front * obstacle_next_length/2
                    my_points_2_start_point = right_point +plan_front*obstacle_length/2
                    my_points_2 = gen_straight_points_p2p(
                        my_points_2_start_point, my_points_2_end_point, 5, endpoint=True)

                    # Replace points in plan with my_points.

                    my_tolerance = np.ones(
                        my_points_2.shape[0], dtype="float32") * 0.05
                    # Set critical tolerance (default: False). This is only noticable on hardware.
                    my_critical_tolerance = np.ones(
                        my_points_2.shape[0], dtype="bool")

                    start_dom = my_dom + 0.75 + obstacle_length/2

                    end_dom = obstacle_next_start_dom + obstacle_next_length/2

                    plan.replace_points_by_dist_on_map(
                        start_dom, end_dom, my_points_2,
                        left_tolerance=my_tolerance,
                        left_tolerance_critical=my_critical_tolerance
                    )

                    # Plan was changed now.
                    # To visualize the difference between map and plan data we get another point from both.
                    # 0.50 meters in front of where we started to change the plan.
                    dom = my_dom + 0.75

                    # Get map point at that distance_on_map.
                    map_loc: MapLocation = map_w.locate_distance_on_map(dom)
                    left_map_point = map_loc.left_lane_point

                    # Get plan point at that distance_on_map.
                    idx = plan.determine_idx_before_distance_on_map(dom)
                    plan_point = plan.points[idx]

                    # Show markers.
                    show_marker(
                        4, left_map_point[0], left_map_point[1], 0, 1, 0)
                    show_marker(5, plan_point[0], plan_point[1], 0, 1, 1)

                else:
                    
                    end_point3 = left_point + plan_front*(obstacle_length + 0.75)
                    end_point3_tangent = map_tangent
                    #end_point1[0], end_point1[1], tangen_y, end_point3[0], end_point3[1], tangen_y)


                    t0, f0 =0, end_point1[0]
                    t1, f1 =1, end_point3[0]
                    t2, f2 =0, plan_tanget[0]
                    t3, f3 =1, end_point3_tangent[0]
                    a0, b0, c0, d0= compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                    #compute spline for y
                    t0, f0 =0, end_point1[1]
                    t1, f1 =1, end_point3[1]
                    t2, f2 =0, plan_tanget[1]
                    t3, f3 =1, end_point3_tangent[1]
                    a1, b1, c1, d1 = compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                    #compute x and y
                    t= np.linspace(0,1,10)
                    x=a0*t**3 + b0*t**2 + c0*t + d0
                    y=a1*t**3 + b1*t**2 + c1*t + d1

                    my_points_3 = np.array([x,y],dtype="float32").T

                    my_tolerance = np.ones(
                        my_points_3.shape[0], dtype="float32") * 0.05
                    # Set critical tolerance (default: False). This is only noticable on hardware.
                    my_critical_tolerance = np.ones(
                        my_points_3.shape[0], dtype="bool")

                    #######
                    start_dom = my_dom + 0.75 + obstacle_length/2
                    ########
                    # end_dom   = my_dom + distance(my_points_3[0], my_points_3[-1])
                    end_dom = my_dom + 1.5 + obstacle_length
                    plan.replace_points_by_dist_on_map(
                        start_dom, end_dom, my_points_3,
                        left_tolerance=my_tolerance,
                        left_tolerance_critical=my_critical_tolerance
                    )

                    # Plan was changed now.
                    # To visualize the difference between map and plan data we get another point from both.
                    # 0.50 meters in front of where we started to change the plan.
                    dom = my_dom + 0.75

                    # Get map point at that distance_on_map.
                    map_loc: MapLocation = map_w.locate_distance_on_map(dom)
                    left_map_point = map_loc.left_lane_point

                    # Get plan point at that distance_on_map.
                    idx = plan.determine_idx_before_distance_on_map(dom)
                    plan_point = plan.points[idx]

                    # Show markers.
                    show_marker(
                        6, left_map_point[0], left_map_point[1], 0, 1, 0)
                    show_marker(7, plan_point[0], plan_point[1], 0, 1, 1)

            else:
                end_point3 = left_point + plan_front*(obstacle_length + 0.75)
                end_point3_tangent = map_tangent
                #end_point1[0], end_point1[1], tangen_y, end_point3[0], end_point3[1], tangen_y)


                t0, f0 =0, end_point1[0]
                t1, f1 =1, end_point3[0]
                t2, f2 =0, plan_tanget[0]
                t3, f3 =1, end_point3_tangent[0]
                a0, b0, c0, d0= compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                #compute spline for y
                t0, f0 =0, end_point1[1]
                t1, f1 =1, end_point3[1]
                t2, f2 =0, plan_tanget[1]
                t3, f3 =1, end_point3_tangent[1]
                a1, b1, c1, d1 = compute_spline(t0, f0, t1, f1, t2, f2, t3, f3)

                #compute x and y
                t= np.linspace(0,1,10)
                x=a0*t**3 + b0*t**2 + c0*t + d0
                y=a1*t**3 + b1*t**2 + c1*t + d1

                my_points_3 = np.array([x,y],dtype="float32").T




                my_tolerance = np.ones(
                    my_points_3.shape[0], dtype="float32") * 0.05
                # Set critical tolerance (default: False). This is only noticable on hardware.
                my_critical_tolerance = np.ones(
                    my_points_3.shape[0], dtype="bool")

                start_dom = my_dom + 0.75 + obstacle_length/2

                end_dom = my_dom + 1.5 + obstacle_length

                plan.replace_points_by_dist_on_map(
                    start_dom, end_dom, my_points_3,
                    left_tolerance=my_tolerance,
                    left_tolerance_critical=my_critical_tolerance
                )

                # Plan was changed now.
                # To visualize the difference between map and plan data we get another point from both.
                # 0.50 meters in front of where we started to change the plan.
                dom = my_dom + 0.75

                # Get map point at that distance_on_map.
                map_loc: MapLocation = map_w.locate_distance_on_map(dom)
                left_map_point = map_loc.left_lane_point

                # Get plan point at that distance_on_map.
                idx = plan.determine_idx_before_distance_on_map(dom)
                plan_point = plan.points[idx]

                # Show markers.
                show_marker(10, left_map_point[0], left_map_point[1], 0, 1, 0)
                show_marker(11, plan_point[0], plan_point[1], 0, 1, 1)


def show_marker(id, x, y, r=1.0, g=0.0, b=0.0, scale=0.1, a=1.0):
    # Create publisher to publish the marker msg. (Usually done in class, just for debugging here)
    global marker_pub
    if marker_pub is None:
        marker_pub = rospy.Publisher(
            "/my_marker_topic", Marker, queue_size=10, latch=True)

    # Create marker to be published.
    msg = Marker()

    # Set marker data.
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.id = id
    msg.lifetime = rospy.Duration(0.5)  # Show for 0.5 seconds.
    msg.action = Marker.ADD
    msg.type = Marker.SPHERE
    msg.scale.x = scale
    msg.scale.y = scale
    msg.scale.z = scale
    msg.color.r = r
    msg.color.g = g
    msg.color.b = b
    msg.color.a = a

    # Set marker pose.
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.00
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    # Publish marker to show it in rviz.
    marker_pub.publish(msg)
