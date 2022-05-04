#!/usr/bin/env python
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Header

from loewen_msgs.msg import *
from utils.map.MapWrapper import MapWrapper
from waypoint_planning.PlanningHelper import PlanningHelper
from waypoint_planning.execution_stage.PlanExecutor import PlanExecutor
from waypoint_planning.scheduling import create_schedule, CIRCUIT_MODE_SCHEDULING, OBSTACLE_MODE_SCHEDULING
from planning_stage import create_obstacle_mode_plan, create_circuit_mode_plan

from waypoint_planning import config


class PlanningNode:
    """gets a map and car position and generates waypoints"""
    CIRCUIT_MAXIMUM_SPEED_KMH = 0
    OBSTACLE_MAXIMUM_SPEED_KMH = 0

    def __init__(self):
        self.helper = PlanningHelper()
        self.last_schedule = Schedule()
        self.plan_executor = PlanExecutor()
        self.driving_mode = DrivingMode.DRIVING_MODE_STATUS_NONE
        self.blind_spots = None  # type: BlindSpots

        self.map_sub = rospy.Subscriber("/map", numpy_msg(Map), callback=self.map_callback, queue_size=1, tcp_nodelay=True)
        self.blind_spots_sub = rospy.Subscriber("/blind_spots", numpy_msg(BlindSpots), callback=self.blind_spots_callback, queue_size=1, tcp_nodelay=True)
        self.plan_pub = rospy.Publisher("/waypoint_plan", numpy_msg(WaypointPlan), queue_size=1, tcp_nodelay=True)
        self.state_pub = rospy.Publisher("/waypoint_plan/state", WaypointPlanState, queue_size=1, tcp_nodelay=True)
        self.schedule_pub = rospy.Publisher("/schedule", Schedule, queue_size=1, latch=True)
        self.schedule_pub.publish(Schedule(enabled_detectors=[Schedule.DETECTOR_LANE]))

        self.time_started = rospy.Time.now()
        self.drive_mod_sub = rospy.Subscriber("/driving_mode", DrivingMode, callback=self.drive_mode_callback)
        config.start_servers()

    def drive_mode_callback(self, driving_mode):
        # type: (DrivingMode) -> None
        """
        Called each time the driving mode is changed (i.e. button pressed on car). A driving mode change implies a complete reset.
        :param driving_mode: {DrivingMode} object
        """
        self.helper.set_turn_indicator()
        self.plan_executor.reset()
        if driving_mode.driving_mode != self.driving_mode:
            self.driving_mode = driving_mode.driving_mode
            rospy.logwarn_throttle(5, "Change driving mode")

    def blind_spots_callback(self, blind_spots_msg):
        self.blind_spots = blind_spots_msg

    def map_callback(self, map_msg):
        # type: (Map) -> None
        """this function is called when a new map is received. Based on drive mode a planner is selected and actions
        are generated. Forward the actions to actionPlayer and publish the waypoint plan """
        if len(map_msg.street_map.street.speed_limits_kmh) == 0:
            map_msg.street_map.street.speed_limits_kmh = np.repeat(1000, len(map_msg.street_map.street.distances_on_map))
        if self.driving_mode == DrivingMode.DRIVING_MODE_STATUS_OBSTACLES:
            map_msg.street_map.street.speed_limits_kmh = np.repeat(config.obstacle_mode.max_speed * 36., len(map_msg.street_map.street.distances_on_map))

        map_w = MapWrapper(map_msg)
        if map_w.is_empty:
            rospy.logwarn_throttle(5, "Empty map, skip behavior planning")
            schedule = Schedule()
            schedule.enabled_detectors = [Schedule.DETECTOR_LANE]
            self.schedule_pub.publish(schedule)
            return

        if map_msg.header.frame_id == "":
            map_msg.header.frame_id = "map"

        car_state = self.helper.determine_car_state(map_w)

        if self.driving_mode == DrivingMode.DRIVING_MODE_STATUS_CIRCUIT:
            plan = create_circuit_mode_plan(map_w, self.plan_executor.progress, car_state)
        elif self.driving_mode == DrivingMode.DRIVING_MODE_STATUS_OBSTACLES:
            plan = create_obstacle_mode_plan(map_w, self.plan_executor.progress, car_state, self.blind_spots)
        else:
            # do nothing if we are in no mode
            self.plan_pub.publish(WaypointPlan(header=Header(frame_id="map", stamp=map_w.msg.header.stamp)))
            rospy.logwarn_throttle(5, "No driving mode, skip behavior planning")
            if self.drive_mod_sub.get_num_connections() == 0 and (rospy.Time.now() - self.time_started).to_sec() > 20:
                rospy.logerr_throttle(5, "No /driving_mode publishers found, switching to obstacle mode by default after 20 seconds.")
                self.driving_mode = DrivingMode.DRIVING_MODE_STATUS_OBSTACLES
            return

        # run action execution stage
        plan, state, indicator_light = self.plan_executor.calc_current_behavior(map_w, plan, car_state)
        plan.car_width = config.general.car_width
        plan.car_length = config.general.car_length
        self.plan_pub.publish(plan)
        self.state_pub.publish(state)
        self.helper.publish_indicator_light(indicator_light)

        schedule = create_schedule(self.driving_mode, state.current_state, map_w, car_state.map_location)

        if schedule is not None:
            removed_detectors = set(self.last_schedule.enabled_detectors).difference(schedule.enabled_detectors)
            added_detectors = set(schedule.enabled_detectors).difference(self.last_schedule.enabled_detectors)
            if len(added_detectors) > 0 or len(removed_detectors) > 0:
                schedule.header.stamp = rospy.Time.now()
                self.schedule_pub.publish(schedule)
                for r in removed_detectors:
                    rospy.logwarn("[Scheduling] Removed " + str(r))
                for a in added_detectors:
                    rospy.logwarn("[Scheduling] Added " + str(a))
                self.last_schedule = schedule


if __name__ == '__main__':
    rospy.init_node("my_waypoint_planning_node")
    rospy.logwarn("initialized")
    node = PlanningNode()
    rospy.spin()
