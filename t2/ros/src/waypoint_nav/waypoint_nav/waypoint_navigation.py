import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(3.0, self.send_waypoints)
        self.waypoints = [
            (2.0, 2.0, 0.0),
            (0.0, 2.0, 1.57),
            (-2.0, -2.0, 3.14)
        ]
        self.index = 0

    def send_waypoints(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("Navigation Complete")
            return

        waypoint = self.waypoints[self.index]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.orientation.z = waypoint[2]

        self.publisher.publish(goal)
        self.get_logger().info(f"Moving to waypoint {self.index + 1}: {waypoint}")
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
