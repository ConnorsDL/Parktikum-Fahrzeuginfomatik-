#!/usr/bin/env python
import rospy
import numpy as np

import waypoint_planning.config as config
from waypoint_planning.Plan import Plan
from waypoint_planning.execution_stage.ExecutionProgress import ExecutionProgress
from waypoint_planning.planning_stage.parking_utils import find_parking_spot, get_approaching_state, get_moving_away_state
from waypoint_planning.predefined_state_sequence.PredefinedStateSequence import PredefinedStateSequence
from waypoint_planning.predefined_state_sequence.PredefinedDrivingState import PredefinedDrivingState
from waypoint_planning.predefined_state_sequence.PredefinedWaitingState import PredefinedWaitingState

from utils.map.MapWrapper import MapWrapper, MapLocation
from utils.geometry import distance, vector_of, angle_of, left_orthogonal, right_orthogonal, normalize
from utils.geometry.street import gen_curve_points_2d, gen_straight_points_2d, gen_straight_points_p2p

from loewen_msgs.msg import WaypointPlanState
from visualization_msgs.msg import Marker


# Helper for debugging. (Publishers are usually defined within a class)
marker_pub = None

def plan_my_parallel_parking_behavior(plan, map_w, progress):
    # type: (Plan, MapWrapper, ExecutionProgress) -> None

    # search a parking spot that is in a parking zone context
    parking_context, spot = find_parking_spot(map_w, is_parallel_parking=True)

    # plan parking behavior if parking spot was found
    if spot is not None:

        # ############################################
        # ####  Compute parking spot information  ####
        # ############################################

        # center location of the parking spot (Check out ros/utils/src/utils/map/MapLocation.py for available variables)
        spot_center_loc = map_w.locate_pos(spot.properties.parking_spot_position)
        # orientation of the parking spot
        spot_ori = angle_of(v=spot.properties.parking_spot_orientation)
        # vectors for movement of points in specific direction
        front = normalize(vector_of(spot_ori))
        left = normalize(left_orthogonal(front))
        right = left * -1.0
        back = front * -1.0
        # length of the car as configured
        car_length = config.general.car_length
        # get length of the parking spot
        spot_length = min(spot.location.length_on_map, car_length + 0.5)

        # ########################################################################
        # ####  Init statemachine that should implement the parking behavior  ####
        # ########################################################################

        # The id of the planned state machine. We use the id of the parking spot so each parking spot can have its own parking state machine
        parking_state_machine_id = WaypointPlanState.PARKING + "_" + str(spot.id)
        # id of the problem we want to solve. All parking state machine in one context solve the same problem (= parking in that context)
        # Therefore, all parking state machines in the same context should have the same problem id
        parking_problem_id = WaypointPlanState.PARKING + "_" + str(parking_context.id)
        # this is where the state machine will be hooked into the main trajectory
        start_distance_on_map = spot.location.start_distance_on_map - (spot_length / 2) - car_length - 1
        # create the state machine
        parking_dist_predefine = spot_length + car_length + 1
        parking_state_sequence = PredefinedStateSequence(sm_id=parking_state_machine_id,
                                                         problem_name=WaypointPlanState.PARKING,
                                                         distance_on_map=start_distance_on_map,
                                                         length_on_map=parking_dist_predefine,
                                                         key_state_id=WaypointPlanState.PARKING_WAIT_IN_SPOT,
                                                         problem_id=parking_problem_id)
        # List of states that implement the parking behavior
        # parking_state_sequence.states = []
        parking_state_sequence.states = []
        # ##########################################################
        # ####  Implement your parallel parking maneuver here!  ####
        # ##########################################################

        # TODO: Delete show_examples().
        # Make sure parking_state_sequence.states is filled with at least one state before deleting or the node will crash.
        show_examples(plan, map_w, spot_center_loc, spot_ori, front, left, right, back, parking_state_sequence)

        # TODO: Calculate relevant points for the trajectory calculation.
        # You can use the functions provided in ros\utils\src\utils\geometry\street.py or create your own geometric calculation functions.
        # Add your individual states to the state machine in proper order.
        # You can find a working example for orthogonal parking in example_parking_behavior.py.

        # ...
        
    

        # ############################################
        # ####  Add state machine to be executed  ####
        # ############################################

        # Add list of states to the plan. (Will crash if list is empty!)
        plan.state_machines.append(parking_state_sequence)


def show_examples(plan, map_w, spot_center_loc, spot_ori, front, left, right, back, parking_state_sequence):

    # TODO: Delete example 1: How to show markers for debugging. (Check out ros/utils/src/utils/map/MapLocation.py for available variables)
    # my_parking_spot_point = spot_center_loc.pos                 # Get point on parking spot from parking spot location.
    # my_right_lane_point   = spot_center_loc.right_lane_point    # Get point on right lane from parking spot location.
    # show_marker(id=1, x=0.0, y=0.0)                                                                 # Mark map coordinate system origin.
    # show_marker(id=2, x=my_parking_spot_point[0], y=my_parking_spot_point[1], r=0.0, g=1.0, b=0.0)  # Mark parking spot point.
    # show_marker(id=3, x=my_right_lane_point[0], y=my_right_lane_point[1], r=0.0, g=0.0, b=1.0)      # Mark right lane point.

    # my_start_point = my_right_lane_point + back  * 1.0          # Define point 1 meter behind the parking spot on the right lane.
    # my_end_point   = my_right_lane_point + front * 1.0          # Define point 1 meter in front of the parking spot on the right lane.
    # show_marker(id=4, x=my_start_point[0], y=my_start_point[1], r=1.0, g=0.0, b=0.0)
    # show_marker(id=5, x=my_end_point[0], y=my_end_point[1], r=0.0, g=1.0, b=0.0)

    # TODO: Delete example 2: How to compute points.
    # radius = distance(my_parking_spot_point, my_start_point)
    # my_points = gen_curve_points_2d(    # Generate points along curvce with:
    #     start       = my_start_point,   #   - start point on the circle
    #     radius      = radius,           #   - radius of the cricle
    #     start_angle = spot_ori,         #   - starting angle of the curve
    #     curve_angle = np.pi/2)          #   - angle to move starting angle along the circle 
    # # Visualize some computed waypoints.
    # num_points = my_points.shape[0]     # Points are in np.ndarray with each row being x,y values of a point.
    # x = my_points[0,0]                  # 1st point x = [0,0] = [row,col]
    # y = my_points[0,1]                  # 1st point y = [0,1] = [row,col]
    # show_marker(10, x, y, g=1)
    # x = my_points[num_points//4*1,0]
    # y = my_points[num_points//4*1,1]
    # show_marker(11, x, y, g=1)
    # x = my_points[num_points//4*2,0]
    # y = my_points[num_points//4*2,1]
    # show_marker(12, x, y, g=1)
    # x = my_points[num_points//4*3,0]
    # y = my_points[num_points//4*3,1]
    # show_marker(13, x, y, g=1)
    # x = my_points[num_points-1,0]
    # y = my_points[num_points-1,1]
    # show_marker(14, x, y, g=1)

    # TODO: Delete example 3: How to add an approaching state.
    # my_approaching_state = get_approaching_state(map_w, my_points[0], plan) # Get approaching state.
    # my_approaching_state.indicator_light.turn_indicator_left = True         # Set indicator light left or right.
    # parking_state_sequence.states.append(my_approaching_state)              # Add state to statemachine.

    # TODO: Delete example 4: How to add a driving state for driving forward.
    # my_parking_in_state1 = PredefinedDrivingState(  # Create PredefinedDrivingState with:
    #     state_name = "my_parking_in_state1",        #   - State name
    #     state_id   = "my_parking_in_state1")        #   - Unique state string id
    # my_parking_in_state1.set_points(                # Add waypoints to the driving state with:
    #     my_points,                                  #   - points to be added
    #     np.linspace(0.1, 0.02, len(my_points)),     #   - left tolerance distance
    #     np.linspace(0.1, 0.02, len(my_points)))     #   - right tolrance distance
    # my_parking_in_state1.target_speed_kmh = config.circuit_mode.parking_max_speed_mps * 36.0    # Set driving speed.
    # my_parking_in_state1.indicator_light.turn_indicator_right = True
    # parking_state_sequence.states.append(my_parking_in_state1)

    # TODO: Delete example 5: How to add a driving state for driving backward.
    # my_parking_out_state2 = PredefinedDrivingState(
    #     state_name = "my_parking_out_state2",
    #     state_id   = "my_parking_out_state2")
    # my_parking_out_state2.set_points(               # Waypoints are added like they would be driven forward.
    #     my_points,                                  # By setting a negative speed they are executed from the end.
    #     np.linspace(0.1, 0.02, len(my_points)),
    #     np.linspace(0.1, 0.02, len(my_points)))
    # my_parking_out_state2.target_speed_kmh = -config.circuit_mode.parking_max_speed_mps * 36.0  # Set negative driving speed.
    # my_parking_out_state2.indicator_light.turn_indicator_right = True
    # parking_state_sequence.states.append(my_parking_out_state2)

    # TODO: Delete example 6: How to add a waiting state.
    # my_waiting_state = PredefinedWaitingState(                  # Create state with:
    #     state_name = "my_waiting_state1",                       #   - state name
    #     time_limit = config.circuit_mode.parking_stop_time,     #   - time to wait
    #     point      = my_start_point,                            #   - position to wait at
    #     tangent    = vector_of(spot_ori),                       #   - orientation to wait at
    #     state_id   = "my_waiting_state1")                       #   - unique state id string
    # my_waiting_state.indicator_light.hazard_light = True        # Flash hazard light while waiting.
    # parking_state_sequence.states.append(my_waiting_state)      # Add state to statemachine.

    # TODO: Delete example 7: How to add move away state.
    # my_move_away_state = get_moving_away_state(map_w, my_points[num_points-1], plan, "my_move_away_state")
    # parking_state_sequence.states.append(my_move_away_state)





        parking_context, spot = find_parking_spot(map_w, is_parallel_parking=True)

    
        # center location of the parking spot (Check out ros/utils/src/utils/map/MapLocation.py for available variables)
        spot_center_loc = map_w.locate_pos(spot.properties.parking_spot_position)
        # orientation of the parking spot
        spot_ori = angle_of(v=spot.properties.parking_spot_orientation)
        # vectors for movement of points in specific direction
        front = normalize(vector_of(spot_ori))
        left = normalize(left_orthogonal(front))
        right = left * -1.0
        back = front * -1.0
        # length of the car as configured
        car_length = config.general.car_length
        # get length of the parking spot
        spot_length = min(spot.location.length_on_map, car_length + 0.5)


    ## 1.Calculate relevant points
        my_parking_spot_point = spot_center_loc.pos                 # Get point on parking spot from parking spot location.
        my_right_lane_point   = spot_center_loc.right_lane_point    # Get point on right lane from parking spot location.
        show_marker(id=1, x=0.0, y=0.0)                                                                 # Mark map coordinate system origin.
        show_marker(id=2, x=my_parking_spot_point[0], y=my_parking_spot_point[1], r=0.0, g=1.0, b=0.0)  # Mark parking spot point.
        show_marker(id=3, x=my_right_lane_point[0], y=my_right_lane_point[1], r=0.0, g=0.0, b=1.0)      # Mark right lane point.

        my_start_point = my_right_lane_point + back  * 0.9          # Define point 1 meter behind the parking spot on the right lane.
        my_end_point   = my_right_lane_point + front * 0.9          # Define point 1 meter in front of the parking spot on the right lane.
        show_marker(id=4, x=my_start_point[0], y=my_start_point[1], r=1.0, g=0.0, b=0.0)
        show_marker(id=5, x=my_end_point[0], y=my_end_point[1], r=0.0, g=1.0, b=0.0)

    ## 2.compute approaching points
        radius = distance(my_end_point, my_parking_spot_point)
        my_points = gen_curve_points_2d(    # Generate points along curvce with:
            start       = my_end_point,   #   - start point on the circle
            radius      = radius*0.75,           #   - radius of the cricle
            # radius      = radius,
            # start_angle = np.pi+spot_ori,         #   - starting angle of the curve
            start_angle = np.pi+spot_ori-np.pi*15/180,
            curve_angle = np.pi*6/18)          #   - angle to move starting angle along the circle 

        
        # Visualize some computed waypoints.
        num_points = my_points.shape[0]     # Points are in np.ndarray with each row being x,y values of a point.
        x = my_points[0,0]                  # 1st point x = [0,0] = [row,col]
        y = my_points[0,1]                  # 1st point y = [0,1] = [row,col]
        show_marker(10, x, y, g=1)
        x = my_points[num_points//4*1,0]
        y = my_points[num_points//4*1,1]
        show_marker(11, x, y, g=1)
        x = my_points[num_points//4*2,0]
        y = my_points[num_points//4*2,1]
        show_marker(12, x, y, g=1)
        x = my_points[num_points//4*3,0]
        y = my_points[num_points//4*3,1]
        show_marker(13, x, y, g=1)
        x = my_points[num_points-1,0]
        y = my_points[num_points-1,1]
        show_marker(14, x, y, g=1)


       
        
        my_parking_state2_startpoint1 = my_points[num_points-1]

        my_parking_state2_startpoint2_distance= (spot_length)/2 - car_length -1
        
        my_parking_state2_startpoint2 = my_parking_spot_point + back * my_parking_state2_startpoint2_distance

        radius = distance(my_parking_state2_startpoint1, my_parking_state2_startpoint2)
        my_parking_state2_point = gen_curve_points_2d(    # Generate points along curvce with:
            start       = my_parking_state2_startpoint1,   #   - start point on the circle
            radius      = radius*3/5,           #   - radius of the cricle
            start_angle = spot_ori-np.pi/2-np.pi/5,         #   - starting angle of the curve
            curve_angle = -np.pi/3.5)          #   - angle to move starting angle along the circle 

        num_points_parking_state2 = my_parking_state2_point.shape[0]     # Points are in np.ndarray with each row being x,y values of a point.
        x = my_parking_state2_point[0,0]                  # 1st point x = [0,0] = [row,col]
        y = my_parking_state2_point[0,1]                  # 1st point y = [0,1] = [row,col]
        show_marker(15, x, y, g=1)
        x = my_parking_state2_point[num_points_parking_state2//4*1,0]
        y = my_parking_state2_point[num_points_parking_state2//4*1,1]
        show_marker(16, x, y, g=1)
        x = my_parking_state2_point[num_points_parking_state2//4*2,0]
        y = my_parking_state2_point[num_points_parking_state2//4*2,1]
        show_marker(17, x, y, g=1)
        x = my_parking_state2_point[num_points_parking_state2//4*3,0]
        y = my_parking_state2_point[num_points_parking_state2//4*3,1]
        show_marker(18, x, y, g=1)
        x = my_parking_state2_point[num_points_parking_state2-1,0]
        y = my_parking_state2_point[num_points_parking_state2-1,1]
        show_marker(19, x, y, g=1)

        
    
        my_way_point=np.vstack((my_points,my_parking_state2_point))
        
        

    
    ## 3.add an approaching state.
        # my_approaching_state = get_approaching_state(map_w, my_points[0], plan) # Get approaching state.
        my_approaching_state = get_approaching_state(map_w, my_way_point[0], plan)
        my_approaching_state.indicator_light.turn_indicator_right = True         # Set indicator light left or right.
        parking_state_sequence.states.append(my_approaching_state)              # Add state to statemachine.


    ## 4.my_parking_in_state1.
      
        my_parking_state1 = PredefinedDrivingState(
            state_name = "my_parking_state1",
            state_id   = "my_parking_state1")
        my_parking_state1.set_points(               # Waypoints are added like they would be driven forward.
            #my_points[::-1],                                  # By setting a negative speed they are executed from the end.
            # np.linspace(0.02, 0.1, len(my_points)),
            # np.linspace(0.02, 0.1, len(my_points)))
            
            my_way_point[::-1], 
            np.linspace(0.02, 0.1, len(my_way_point)),
            np.linspace(0.02, 0.1, len(my_way_point)))
        my_parking_state1.target_speed_kmh = -config.circuit_mode.parking_max_speed_mps * 36.0  # Set negative driving speed.
        my_parking_state1.indicator_light.turn_indicator_right = True
        parking_state_sequence.states.append(my_parking_state1)

    # ## 5.my_parking_in_state2.
      
    #     my_parking_state2 = PredefinedDrivingState(
    #         state_name = "my_parking_state2",
    #         state_id   = "my_parking_state2")
    #     my_parking_state2.set_points(               # Waypoints are added like they would be driven forward.
    #         my_parking_state2_point[::-1],                                  # By setting a negative speed they are executed from the end.
    #         np.linspace(0.02, 0.1, len(my_parking_state2_point)),
    #         np.linspace(0.02, 0.1, len(my_parking_state2_point)))
    #     my_parking_state2.target_speed_kmh = -config.circuit_mode.parking_max_speed_mps * 36.0  # Set negative driving speed.
    #     my_parking_state2.indicator_light.turn_indicator_right = True
    #     parking_state_sequence.states.append(my_parking_state2)
       
        
    ## 6.waiting_state 
        my_waiting_state2 = PredefinedWaitingState(                  # Create state with:
            state_name = "my_waiting_state2",                       #   - state name
            time_limit = config.circuit_mode.parking_stop_time,     #   - time to wait
            # point      = my_points[num_points-1],               #   - position to wait at 
            # point = my_way_point[num_points-1],
            point = my_way_point[len(my_way_point)-1],
            tangent    = vector_of(spot_ori),                       #   - orientation to wait at
            state_id   = "my_waiting_state2")                       #   - unique state id string
        my_waiting_state2.indicator_light.hazard_light = True        # Flash hazard light while waiting.
        parking_state_sequence.states.append(my_waiting_state2)      # Add state to statemachine.

    ## 7.move_away_state
        my_move_away_state = get_moving_away_state(map_w, my_way_point[len(my_way_point)-1], plan, "my_move_away_state")
        # my_move_away_state = get_moving_away_state(map_w, my_parking_state2_point[num_points-1], plan, "my_move_away_state")
        my_move_away_state.indicator_light.turn_indicator_left = True
        parking_state_sequence.states.append(my_move_away_state)


def show_marker(id, x, y, r=1.0, g=0.0, b=0.0, scale=0.1):
    # Create publisher to publish the marker msg. (Usually done in class, just for debugging here)
    global marker_pub
    if marker_pub is None:
        marker_pub = rospy.Publisher("/my_marker_topic", Marker, queue_size=10, latch=True)

    # Create marker to be published.
    msg = Marker()

    # Set marker data.
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.id = id
    msg.lifetime = rospy.Duration(0.5) # Show for 0.5 seconds.
    msg.action = Marker.ADD
    msg.type = Marker.SPHERE
    msg.scale.x = scale
    msg.scale.y = scale
    msg.scale.z = scale
    msg.color.r = r
    msg.color.g = g
    msg.color.b = b
    msg.color.a = 1.0

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

