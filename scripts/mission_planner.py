#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Traffic-light and object mission supervisor for the MORAI mini mission."""

import math

import rospy
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, ObjectStatusList
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


class MissionPlanner:
    RED = 1
    YELLOW = 4
    GREEN = 16
    LEFT = 32

    def __init__(self):
        rospy.init_node("mission_planner")

        self.cruise_speed = float(rospy.get_param("~cruise_speed_kph", 30.0))
        self.yellow_go_distance = float(rospy.get_param("~yellow_go_distance_m", 8.0))
        self.slowdown_distance = float(rospy.get_param("~slowdown_distance_m", 35.0))
        self.stop_margin = float(rospy.get_param("~stop_margin_m", 2.5))
        self.object_stop_distance = float(rospy.get_param("~object_stop_distance_m", 12.0))
        self.object_lateral_margin = float(rospy.get_param("~object_lateral_margin_m", 1.8))
        self.finish_slowdown_distance = float(rospy.get_param("~finish_slowdown_distance_m", 15.0))
        self.signal_timeout = float(rospy.get_param("~signal_timeout_sec", 1.0))
        self.stop_lines = self._load_stop_lines(rospy.get_param("~stop_lines", []))

        self.path = None
        self.status = None
        self.objects = []
        self.signal_status = {}
        self.signal_update_time = {}
        self.current_waypoint = -1
        self.path_s = []

        self.speed_pub = rospy.Publisher("/mission_target_speed", Float32, queue_size=1)
        self.state_pub = rospy.Publisher("/mission_state", String, queue_size=1)
        rospy.Subscriber("/global_path", Path, self.path_callback, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback, queue_size=1)
        rospy.Subscriber(
            "/GetTrafficLightStatus",
            GetTrafficLightStatus,
            self.traffic_callback,
            queue_size=10,
        )
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.loginfo("mission_planner started: %d stop line(s)", len(self.stop_lines))
        rospy.spin()

    @staticmethod
    def _load_stop_lines(raw):
        result = []
        for item in raw:
            try:
                result.append(
                    {
                        "signal_id": str(item["signal_id"]),
                        "waypoint": int(item["waypoint"]),
                        "maneuver": str(item.get("maneuver", "straight")).lower(),
                    }
                )
            except (KeyError, TypeError, ValueError):
                rospy.logwarn("Ignoring invalid stop line parameter: %s", item)
        return result

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.path = msg
        self.path_s = [0.0]
        for i in range(1, len(msg.poses)):
            p0 = msg.poses[i - 1].pose.position
            p1 = msg.poses[i].pose.position
            self.path_s.append(self.path_s[-1] + math.hypot(p1.x - p0.x, p1.y - p0.y))

    def status_callback(self, msg):
        self.status = msg

    def traffic_callback(self, msg):
        signal_id = str(msg.trafficLightIndex)
        self.signal_status[signal_id] = int(msg.trafficLightStatus)
        self.signal_update_time[signal_id] = rospy.Time.now()

    def object_callback(self, msg):
        # Static obstacles are handled by the lattice planner. The supervisor
        # reserves emergency braking for pedestrians and moving NPC vehicles.
        moving_npcs = [
            obj for obj in msg.npc_list
            if math.hypot(obj.velocity.x, obj.velocity.y) > 0.3
        ]
        self.objects = list(msg.pedestrian_list) + moving_npcs

    def _nearest_waypoint(self):
        if self.path is None or self.status is None:
            return -1
        x = self.status.position.x
        y = self.status.position.y
        start = max(0, self.current_waypoint - 20)
        end = min(len(self.path.poses), max(start + 1, self.current_waypoint + 200))
        if self.current_waypoint < 0:
            start, end = 0, len(self.path.poses)
        return min(
            range(start, end),
            key=lambda i: (self.path.poses[i].pose.position.x - x) ** 2
            + (self.path.poses[i].pose.position.y - y) ** 2,
        )

    def _signal_allows(self, status, maneuver, distance):
        if maneuver == "left":
            return bool(status & self.LEFT)
        if status & self.GREEN:
            return True
        if status & self.YELLOW and distance <= self.yellow_go_distance:
            return True
        return False

    def _traffic_limit(self):
        limit = self.cruise_speed
        state = "CRUISE"
        now = rospy.Time.now()

        for line in self.stop_lines:
            idx = line["waypoint"]
            if idx <= self.current_waypoint or idx >= len(self.path_s):
                continue
            distance = self.path_s[idx] - self.path_s[self.current_waypoint]
            if distance > self.slowdown_distance:
                continue

            signal_id = line["signal_id"]
            update = self.signal_update_time.get(signal_id)
            # Fail safe: near a configured stop line, stale/missing signal means stop.
            signal = self.signal_status.get(signal_id, 0)
            fresh = update is not None and (now - update).to_sec() <= self.signal_timeout
            if fresh and self._signal_allows(signal, line["maneuver"], distance):
                continue

            if distance <= self.stop_margin:
                candidate = 0.0
                candidate_state = "STOP_RED"
            else:
                # Smoothly reduce 30 -> 0 km/h over the approach distance.
                ratio = min(1.0, max(0.0, (distance - self.stop_margin)
                                     / max(0.1, self.slowdown_distance - self.stop_margin)))
                candidate = self.cruise_speed * math.sqrt(ratio)
                candidate_state = "APPROACH_RED"
            if candidate < limit:
                limit, state = candidate, candidate_state

        return limit, state

    def _object_limit(self):
        """Emergency stop for an object close to the reference path ahead."""
        if not self.objects or self.current_waypoint < 0:
            return self.cruise_speed, "CRUISE"

        end = min(len(self.path.poses), self.current_waypoint + 100)
        closest_forward = float("inf")
        for obj in self.objects:
            half_width = max(0.0, float(obj.size.y) * 0.5)
            threshold = self.object_lateral_margin + half_width
            for i in range(self.current_waypoint, end):
                point = self.path.poses[i].pose.position
                if math.hypot(point.x - obj.position.x, point.y - obj.position.y) <= threshold:
                    distance = self.path_s[i] - self.path_s[self.current_waypoint]
                    closest_forward = min(closest_forward, distance)
                    break

        if closest_forward <= self.object_stop_distance:
            return 0.0, "EMERGENCY_STOP"
        if closest_forward <= self.object_stop_distance * 2.0:
            return 10.0, "OBJECT_SLOW"
        return self.cruise_speed, "CRUISE"

    def _finish_limit(self):
        remaining = self.path_s[-1] - self.path_s[self.current_waypoint]
        if remaining >= self.finish_slowdown_distance:
            return self.cruise_speed, "CRUISE"
        if remaining <= 1.0:
            return 0.0, "FINISH"
        return self.cruise_speed * remaining / self.finish_slowdown_distance, "FINISH"

    def timer_callback(self, _event):
        if self.path is None or self.status is None or not self.path_s:
            return
        self.current_waypoint = self._nearest_waypoint()
        if self.current_waypoint < 0:
            return

        candidates = [self._traffic_limit(), self._object_limit(), self._finish_limit()]
        speed, state = min(candidates, key=lambda item: item[0])
        self.speed_pub.publish(Float32(data=max(0.0, speed)))
        self.state_pub.publish(String(data=state))
        rospy.loginfo_throttle(
            1.0, "mission=%s waypoint=%d target=%.1f km/h",
            state, self.current_waypoint, speed
        )


if __name__ == "__main__":
    try:
        MissionPlanner()
    except rospy.ROSInterruptException:
        pass
