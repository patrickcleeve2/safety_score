#!/usr/bin/env python3


import rospy
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaRadarMeasurement
from sensor_msgs.msg import Imu, Image, CompressedImage 
from safety_score.msg import SafetyScore

import numpy as np
from utils import *


class SafetyScoreNode(object):

    def __init__(self) -> None:
        super().__init__()

        self.velocity = 0.0
        self.fcw = False
        
        imu_sub = rospy.Subscriber("/carla/hero/imu/imu1", Imu, self.imu_callback)
        radar_sub = rospy.Subscriber("/carla/hero/radar/front/radar", CarlaRadarMeasurement, self.radar_callback)
        status_sub = rospy.Subscriber("/carla/hero/vehicle_status", CarlaEgoVehicleStatus, self.status_callback)

        self.score_pub = rospy.Publisher("/carla/hero/safety_score", SafetyScore, queue_size=1)

    def status_callback(self, msg):

        # rospy.loginfo(f"Velocity: {msg.velocity:.4f} m/s")
        self.velocity = msg.velocity

    def radar_callback(self, msg):

        distances = []
        for det in msg.detections:
            # filter to detections right infront of vehicle, and moving, (magic numbers)
            if abs(det.azimuth) < 0.01 and abs(det.altitude) < 0.02 and abs(det.velocity) > 0.01 and det.depth < 50:  

                distances.append(det.depth)

        if distances:
            self.avg_distance = np.mean(distances)
            
            # time to collision
            self.time_to_collision = self.avg_distance / (self.velocity + 0.001)
            rospy.loginfo(f"Lead Vehicle Distance: {self.avg_distance:.4f} m")
            rospy.loginfo(f"Time to Collision: {self.time_to_collision:.4f} sec")
            
            # forward collision warning 
            if self.time_to_collision < 0.5:
                rospy.loginfo(f"FORWARD COLLISION WARNING")
                self.fcw = True

            rospy.loginfo("-"*50)
        else:
            self.avg_distance = 9999
            self.time_to_collision = 9999
            self.fcw = False

        self.publish_safety_score()
        # TODO: move the actual calculation here instead of external...


    def imu_callback(self, msg):

        # deceleration only
        if msg.linear_acceleration.x <= -LONGITUDINAL_ACCEL_THRESHOLD:
            rospy.loginfo(f"HARD BRAKING (Longitudinal): {msg.linear_acceleration.x:.4f} m/s2")
            rospy.loginfo("-"*50)


        if abs(msg.linear_acceleration.y) >= LATERAL_ACCEL_THRESHOLD:
            rospy.loginfo(f"AGGRESIVE TURNING (Lateral): {msg.linear_acceleration.y:.4f} m/s2")
            rospy.loginfo("-"*50)

        self.linear_acceleration_x = msg.linear_acceleration.x
        self.linear_acceleration_y = msg.linear_acceleration.y

    def publish_safety_score(self):
        
        msg = SafetyScore()
        msg.header.stamp = rospy.Time.now()
        msg.velocity = self.velocity
        msg.average_distance = self.avg_distance
        msg.time_to_collision = self.time_to_collision
        msg.forward_collision_warning = self.fcw
        msg.linear_acceleration_x = self.linear_acceleration_x
        msg.linear_acceleration_y = self.linear_acceleration_y 

        self.score_pub.publish(msg)


if __name__ == "__main__":

    rospy.init_node("safety_score_node")
    rospy.loginfo("Hello Safety Score")

    safety_score = SafetyScoreNode()

    rospy.spin()

# TODO: add distance travelled to msg