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
from utils.geometry import distance, vector_of, angle_of, left_orthogonal, normalize
from utils.geometry.street import gen_curve_points_2d

from loewen_msgs.msg import ObjectClass, WaypointPlanState, VerifiedObject, Context


def plan_example_parking_behavior(plan, map_w, progress):
    # type: (Plan, MapWrapper, ExecutionProgress) -> None

    # search a parking spot that is in a parking zone context
    parking_context, spot = find_parking_spot(map_w)

    # plan parking behavior if parking spot was found
    if spot is not None:
        # get parking spot orientation
        spot_vec = normalize(spot.properties.parking_spot_orientation)
        spot_ori = angle_of(spot_vec)

        # compute target parking position
        parking_target_offset = config.circuit_mode.parking_offset_longitudinal * spot_vec + config.circuit_mode.parking_offset_lateral * left_orthogonal(spot_vec)
        target = spot.properties.parking_spot_position + parking_target_offset

        # compute waypoints towards the parking spot
        spot_loc = map_w.locate_pos(target)
        radius = max(0.2, distance(target, spot_loc.right_lane_point) - 0.2)
        parking_points = gen_curve_points_2d(target - vector_of(spot_ori) * 0.2, radius, spot_ori + np.pi, -np.pi/2, 15)[::-1]
        parking_points = np.append(parking_points, np.linspace(target - vector_of(spot_ori) * 0.1, target, num=4, endpoint=False), axis=0)

        # add approaching state to state machine
        approaching_state = get_approaching_state(map_w, parking_points[0], plan)
        approaching_state.indicator_light.turn_indicator_left = True

        # add state for turning into the parking spot to state machine
        parking_in_state = PredefinedDrivingState(state_name=WaypointPlanState.PARKING_IN)
        parking_in_state.set_points(
            parking_points,
            np.linspace(0.1, 0.02, len(parking_points)),
            np.linspace(0.1, 0.02, len(parking_points)))
        parking_in_state.target_speed_kmh = config.circuit_mode.parking_max_speed_mps * 36.0
        parking_in_state.indicator_light.turn_indicator_left = True

        # add waiting state to state machine in order to wait a predefined amount of time and flash the hazard lights once
        waiting_state = PredefinedWaitingState(
            WaypointPlanState.PARKING_WAIT_IN_SPOT,
            config.circuit_mode.parking_stop_time,
            target,
            spot_vec,
            state_id=WaypointPlanState.PARKING_WAIT_IN_SPOT)
        waiting_state.indicator_light.hazard_light = True

        # add a state for parking out of the spot to state machine
        parking_out_state = PredefinedDrivingState(WaypointPlanState.PARKING_OUT)
        parking_out_state.set_points(
            parking_in_state.points,
            parking_in_state.left_tolerances,
            parking_in_state.right_tolerances)
        parking_out_state.add_point(target)
        parking_out_state.target_speed_kmh = -config.circuit_mode.parking_max_speed_mps * 36.0
        parking_out_state.indicator_light.turn_indicator_left = True

        # The id of the planned state machine. We use the id of the parking spot so each parking spot can have its own parking state machine.
        parking_state_machine_id = WaypointPlanState.PARKING + "_" + str(spot.id)

        # id of the problem that we want to solve. All parking state machine in one context solve the same problem (= parking in that context).
        # Therefore, all parking state machines in the same context should have the same problem id.
        parking_problem_id = WaypointPlanState.PARKING + "_" + str(parking_context.id)

        # this is where the state machine starts = right before the turning point
        start_distance_on_map = spot.location.start_distance_on_map - radius - 0.3
        # create the state machine
        parking_state_sequence = PredefinedStateSequence(parking_state_machine_id, WaypointPlanState.PARKING, start_distance_on_map,
                                                         key_state_id=WaypointPlanState.PARKING_WAIT_IN_SPOT, problem_id=parking_problem_id)

        parking_state_sequence.states = [approaching_state, parking_in_state, waiting_state, parking_out_state]

        plan.state_machines.append(parking_state_sequence)
