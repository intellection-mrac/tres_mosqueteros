#!/usr/bin/env python

import rospy
import csv
import sys
import os
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math

class TurtleCSVMover:
    def __init__(self):
        rospy.init_node('csv_turtle', anonymous=True)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.teleport_srv = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.current_pose = Pose()
        self.points = []
        self.rate = rospy.Rate(10)

        # Get CSV file path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file = os.path.join(script_dir, 'points.csv')

        # Wait for CSV file to be generated (dynamic mode)
        if not rospy.get_param('~use_static_csv', False):
            rospy.loginfo(f"Waiting for CSV file at {csv_file}...")
            while not os.path.exists(csv_file) and not rospy.is_shutdown():
                time.sleep(1)
            if rospy.is_shutdown():
                sys.exit(1)
            rospy.loginfo(f"CSV file found at {csv_file}")

        self.load_csv(csv_file)
        self.move_to_points()

    def load_csv(self, csv_file):
        raw_points = []
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                next(reader, None)  # Skip header
                for row in reader:
                    if len(row) >= 3:
                        try:
                            x, y = float(row[1]), float(row[2])
                            raw_points.append((x, y))
                        except ValueError:
                            rospy.logwarn(f"Invalid data in row: {row}")
                    else:
                        rospy.logwarn(f"Row too short: {row}")
            if not raw_points:
                rospy.logerr("No valid points loaded from CSV")
                sys.exit(1)

            # Compute bounding box and scale
            x_vals, y_vals = zip(*raw_points)
            min_x, max_x = min(x_vals), max(x_vals)
            min_y, max_y = min(y_vals), max(y_vals)
            safe_min, safe_max = 0.5, 10.5
            safe_range = safe_max - safe_min
            x_range = max_x - min_x if max_x != min_x else 1.0
            y_range = max_y - min_y if max_y != min_y else 1.0
            scale_x = safe_range / x_range
            scale_y = safe_range / y_range
            scale = min(scale_x, scale_y)
            offset_x = safe_min + (safe_range - x_range * scale) / 2 - min_x * scale
            offset_y = safe_min + (safe_range - y_range * scale) / 2 - min_y * scale

            for x, y in raw_points:
                new_x = x * scale + offset_x
                new_y = y * scale + offset_y
                self.points.append((new_x, new_y))
                rospy.loginfo(f"Mapped ({x}, {y}) to ({new_x:.2f}, {new_y:.2f})")

            rospy.loginfo(f"Loaded and scaled {len(self.points)} points")
        except Exception as e:
            rospy.logerr(f"Failed to load CSV: {e}")
            sys.exit(1)

    def pose_callback(self, data):
        self.current_pose = data

    def move_to_point(self, x, y):
        try:
            self.teleport_srv(x, y, 0)
            rospy.loginfo(f"Moved to ({x:.2f}, {y:.2f})")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Teleport failed: {e}")

    def move_to_points(self):
        rospy.wait_for_service('/turtle1/teleport_absolute')
        for x, y in self.points:
            self.move_to_point(x, y)
            self.rate.sleep()
        rospy.loginfo("Finished moving to all points")
        rospy.spin()

if __name__ == '__main__':
    try:
        TurtleCSVMover()
    except rospy.ROSInterruptException:
        pass
