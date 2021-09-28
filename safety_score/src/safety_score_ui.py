#!/usr/bin/env python3

import rospy
from safety_score.msg import SafetyScore

from utils import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *
import sys

class SafetyScoreUI(QMainWindow):
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
        self.safety_score = 0

        # ui
        self.setup_ui()

        # subscriber
        rospy.Subscriber("/carla/hero/safety_score", SafetyScore, self.score_callback)


    
    def setup_ui(self):
        # background colour
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)

        # refreshable parts
        self.refresh_layout()

        # main window
        self.setGeometry(300, 300, 600, 600)
        self.setWindowTitle("Safety Score UI")
        self.show()

    def refresh_layout(self):

        self.vbox = QVBoxLayout()
        # self.vbox.addStretch(0)
        header_label = QLabel()
        header_label.setText("CARLA Safety Score")
        header_label.setStyleSheet("color: white")
        header_label.setAlignment(Qt.AlignCenter)
        header_label.setFont(QFont("Arial", 32, weight=QFont.Bold))
        header_label.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.vbox.addWidget(header_label)
        
        self.score_labels = []
        for i in range(6):
            label = QLabel()
            label.setStyleSheet("background-color: green")
            label.setFont(QFont("Arial", 24))
            label.setAlignment(Qt.AlignCenter)
            label.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
            self.score_labels.append(label)
            self.vbox.addWidget(label)

        # there is probably a better way to do this
        if self.hard_brake_score >= 0.5:
            self.score_labels[0].setStyleSheet("background-color: red")
        elif self.hard_brake_score >= 0.2:
            self.score_labels[0].setStyleSheet("background-color: yellow")
        self.score_labels[0].setText(f"Hard Braking: {self.hard_brake_score:.3f}")

        if self.agg_turning_score >= 0.6:
            self.score_labels[1].setStyleSheet("background-color: red")
        elif self.agg_turning_score >= 0.2:
            self.score_labels[1].setStyleSheet("background-color: yellow")
        self.score_labels[1].setText(f"Aggressive Turning: {self.agg_turning_score:.3f}")

        if self.unsafe_follow_score >= 0.6:
            self.score_labels[2].setStyleSheet("background-color: red")
        elif self.unsafe_follow_score >= 0.2:
            self.score_labels[2].setStyleSheet("background-color: yellow")
        self.score_labels[2].setText(f"Unsafe Following: {self.unsafe_follow_score:.3f}")

        if self.fcw_score > 0.4:
            self.score_labels[3].setStyleSheet("background-color: red")
        self.score_labels[3].setText(f"Forward Collision Warning: {self.fcw_score:.3f}")

        if self.force_disengagement_score == 1.0:
            self.score_labels[4].setStyleSheet("background-color: red")
        self.score_labels[4].setText(f"Forced Disengagement: {self.force_disengagement_score:.3f}")

        if self.safety_score < 60:
            self.score_labels[5].setStyleSheet("background-color: red")
        elif self.safety_score < 90:
            self.score_labels[5].setStyleSheet("background-color: yellow")
        self.score_labels[5].setText(f"Safety Score: {self.safety_score:.2f}")
        self.score_labels[5].setFont(QFont("Arial", 24, weight=QFont.Bold))


        self.main_widget = QWidget()
        self.main_widget.setLayout(self.vbox)
        self.setCentralWidget(self.main_widget)


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
        
        # self.refresh_layout()


def main():
        
    app = QApplication(sys.argv)
    safety_score_ui = SafetyScoreUI()

    timer = QTimer()
    timer.timeout.connect(safety_score_ui.refresh_layout)
    timer.start(1000)

    sys.exit(app.exec_()) 


if __name__ == "__main__":

    rospy.init_node("safety_score_ui")
    rospy.loginfo("hello safety score ui")

    main()
