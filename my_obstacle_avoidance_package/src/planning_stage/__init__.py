import numpy as np
from utils.map.MapWrapper import MapWrapper

from loewen_msgs.msg import BlindSpots

from waypoint_planning.CarState import CarState
from waypoint_planning.execution_stage.ExecutionProgress import ExecutionProgress
from waypoint_planning.planning_stage.intersections import plan_intersection_behavior
from waypoint_planning.planning_stage.parallel_parking import plan_parallel_parking_behavior
from waypoint_planning.planning_stage.parking import plan_parking_behavior
from waypoint_planning.planning_stage.contexts import plan_speed_zone_contexts
from waypoint_planning.planning_stage.crosswalk import plan_crosswalk_behavior
from waypoint_planning.planning_stage.parking_utils import reduce_speed_limits_based_on_parking_contexts
from waypoint_planning.Plan import Plan
import waypoint_planning.config as config

from planning_stage.my_obstacle_avoidance import plan_my_obstacle_avoidance


def create_obstacle_mode_plan(map_w, progress, car_state, blind_spots=None):
    # type: (MapWrapper, ExecutionProgress, CarState, BlindSpots) -> Plan
    """Create actions for obstacle mode
    :type map_w: MapWrapper
    """
    # Create plan for left lane driving
    plan = Plan(map_w, drive_on_right_lane=False)
    # Add obstacle and barred area avoidance behavior to plan.
    plan_my_obstacle_avoidance(plan, map_w)
    # Add crosswalk behavior to plan.
    plan_crosswalk_behavior(plan, map_w)
    # Add intersection behavior to plan.
    plan_intersection_behavior(plan, map_w)
    # Add speed zone context behavior to plan.
    plan_speed_zone_contexts(plan, map_w)
    # Reduce speed for blind spots.
    if blind_spots is not None:
        reduce_speed_in_blind_spots(plan, blind_spots)
    # Return final plan.
    return plan


def create_circuit_mode_plan(map_w, progress, car_state):
    # type: (MapWrapper, ExecutionProgress, CarState) -> Plan
    """Create actions for circuit mode with parking
    :type map_w: MapWrapper
    """
    plan = Plan(map_w)
    reduce_speed_limits_based_on_parking_contexts(plan, map_w, progress)
    if config.circuit_mode.parking_left:
        plan_parking_behavior(plan, map_w, progress)
    if config.circuit_mode.parking_right:
        plan_parallel_parking_behavior(plan, map_w, progress)
    return plan


def reduce_speed_in_blind_spots(plan, blind_spots):
    reduce_speed_mask = np.zeros(shape=(len(plan.points),), dtype=np.bool)
    match_idx1 = np.searchsorted(blind_spots.distances_on_map, plan.distances_on_map[0])
    if match_idx1 == 0:
        match_idx2 = np.searchsorted(plan.distances_on_map, blind_spots.distances_on_map[0])
    else:
        match_idx2 = 0
    num = min(len(blind_spots.distances_on_map) - match_idx1, len(plan.distances_on_map) - match_idx2)
    reduce_speed_mask[match_idx2:match_idx2 + num] = blind_spots.blind[match_idx1:match_idx1 + num]
    plan.reduce_speed_limit_kmh_by_mask(0.5 * 3.6 * 10, reduce_speed_mask)
