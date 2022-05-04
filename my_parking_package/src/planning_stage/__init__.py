from utils.map.MapWrapper import MapWrapper

from loewen_msgs.msg import BlindSpots

from waypoint_planning.CarState import CarState
from waypoint_planning.Plan import Plan
from waypoint_planning.execution_stage.ExecutionProgress import ExecutionProgress
from waypoint_planning.planning_stage.intersections import plan_intersection_behavior
from waypoint_planning.planning_stage.contexts import plan_speed_zone_contexts
from waypoint_planning.planning_stage.obstacles import plan_obstacle_and_barred_area_behavior
from waypoint_planning.planning_stage.crosswalk import plan_crosswalk_behavior
import waypoint_planning.config as config
from waypoint_planning.planning_stage.parking_utils import reduce_speed_limits_based_on_parking_contexts

from planning_stage.example_parking_behavior import plan_example_parking_behavior
from planning_stage.my_parallel_parking_behavior import plan_my_parallel_parking_behavior


def create_obstacle_mode_plan(map_w, progress, car_state, blind_spots=None):
    # type: (MapWrapper, ExecutionProgress, CarState, BlindSpots) -> Plan
    """Create actions for obstacle mode
    :type map_w: MapWrapper
    """
    plan = Plan(map_w)
    plan_obstacle_and_barred_area_behavior(plan, map_w, car_state, blind_spots)
    plan_crosswalk_behavior(plan, map_w)
    plan_intersection_behavior(plan, map_w)
    plan_speed_zone_contexts(plan, map_w)
    return plan

def create_circuit_mode_plan(map_w, progress, car_state):
    # type: (MapWrapper, ExecutionProgress, CarState) -> Plan
    """Create actions for circuit mode with parking
    :type map_w: MapWrapper
    """
    plan = Plan(map_w)
    reduce_speed_limits_based_on_parking_contexts(plan, map_w, progress)
    if config.circuit_mode.parking_left:
        plan_example_parking_behavior(plan, map_w, progress)
    if config.circuit_mode.parking_right:
        plan_my_parallel_parking_behavior(plan, map_w, progress)
    return plan
