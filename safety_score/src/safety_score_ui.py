#!/usr/bin/env python3

import rospy
from safety_score.msg import SafetyScore
import time
from utils import *
from safety_score_node import (
    LATERAL_ACCEL_BASELINE,
    LATERAL_ACCEL_THRESHOLD,
    LONGITUDINAL_ACCEL_BASELINE,
    LONGITUDINAL_ACCEL_THRESHOLD,
    UNSAFE_FOLLOWING_BASELINE,
    UNSAFE_FOLLOWING_THRESHOLD,
)


class SafetyScoreUI(object):
    def __init__(self) -> None:
        super().__init__()

        self.hard_brake_baseline_count = 1e-9
        self.hard_brake_threshold_count = 0
        self.hard_brake_score = 0
        self.agg_turning_baseline_count = 1e-9
        self.agg_turning_threshold_count = 0
        self.agg_turning_score = 0
        self.unsafe_follow_baseline_count = 1e-9
        self.unsafe_follow_threshold_count = 0
        self.unsafe_follow_score = 0
        self.fcw_count = 0
        self.fcw_score = 0
        self.force_disengagement_score = 0
        self.total_messages = 0

        score_sub = rospy.Subscriber(
            "/carla/hero/safety_score", SafetyScore, self.score_callback
        )

    def score_callback(self, msg):

        # hard braking
        if msg.linear_acceleration_x < -LONGITUDINAL_ACCEL_BASELINE:
            self.hard_brake_baseline_count += 1
            if msg.linear_acceleration_x < -LONGITUDINAL_ACCEL_THRESHOLD:
                self.hard_brake_threshold_count += 1

        # aggressive turning
        if abs(msg.linear_acceleration_y) > LATERAL_ACCEL_BASELINE:
            self.agg_turning_baseline_count += 1
            if abs(msg.linear_acceleration_y) > LATERAL_ACCEL_THRESHOLD:
                self.agg_turning_threshold_count += 1

        # unsafe following
        if msg.time_to_collision < UNSAFE_FOLLOWING_BASELINE:
            self.unsafe_follow_baseline_count += 1
            if msg.time_to_collision < UNSAFE_FOLLOWING_THRESHOLD:
                self.unsafe_follow_threshold_count += 1

        # fcw
        if msg.forward_collision_warning:
            self.fcw_count += 1

        # forced disengagment
        if msg.forced_disengagement:
            self.force_disengagement_score = 1

        self.total_messages += 1

        self.hard_brake_score = (self.hard_brake_threshold_count / self.hard_brake_baseline_count)
        self.agg_turning_score = (self.agg_turning_threshold_count / self.agg_turning_baseline_count)
        self.unsafe_follow_score = (self.unsafe_follow_threshold_count / self.unsafe_follow_baseline_count)
        self.fcw_score = self.fcw_count / (self.total_messages * 1e6)  # 1000 msgs per 1000 miles?

        # safety score
        self.pcf = calc_pcf(
            fcw_value=self.fcw_score,
            hb_value=self.hard_brake_score,
            at_value=self.agg_turning_score,
            uf_value=self.unsafe_follow_score,
            fd_value=self.force_disengagement_score,
        )

        self.safety_score = calc_safety_score(self.pcf)

        rospy.loginfo(f"Hard Braking: {self.hard_brake_score:.4f}")
        rospy.loginfo(f"Aggressive Turning: {self.agg_turning_score:.4f}")
        rospy.loginfo(f"Unsafe Following: {self.unsafe_follow_score:.4f}")
        rospy.loginfo(f"Forward Collision Warning: {self.fcw_score:.4f}")
        rospy.loginfo(f"Forced Disengagement: {self.force_disengagement_score:.4f}")
        rospy.loginfo(f"Predicted Collision Frequency: {self.pcf}")
        rospy.loginfo(f"Safety Score: {self.safety_score:.2f}")
        rospy.loginfo("-" * 50)
        # time.sleep(0.25)


if __name__ == "__main__":

    rospy.init_node("safety_score_ui")
    rospy.loginfo("hello safety score ui")

    safety_score_ui = SafetyScoreUI()

    rospy.spin()
