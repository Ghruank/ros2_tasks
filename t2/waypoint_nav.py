#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = int(time.time())
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = yaw  # Simplified; for real angle use quaternion
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Define waypoints here (in map coordinates)
    waypoints = [
        create_pose(0.5, 0.5, 0.0),
        create_pose(1.0, 0.0, 0.0),
        create_pose(0.0, -1.0, 0.0)
    ]

    navigator.goThroughPoses(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f}")
        time.sleep(1.0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Navigation succeeded!")
    else:
        print("Navigation failed or canceled.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

