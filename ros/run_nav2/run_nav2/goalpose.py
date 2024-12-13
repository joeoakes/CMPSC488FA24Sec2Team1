#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__("goal_pose_publisher")

        # Create publisher for goal pose
        self.goal_publisher = self.create_publisher(PoseStamped, "goal_pose", 10)

        self.x = 0.0
        self.y = 1.0
        self.z = 0.0
        self.orientation_w = 1.0

        self.timer = self.create_timer(1.0, self.publish_goal)

        self.get_logger().info("Goal pose publisher has been started")

    def publish_goal(self):
        """Publish a goal pose for Nav2."""
        self.get_logger().info(f"x: {self.x}, y: {self.y}, z:{self.z}")
        goal_pose = PoseStamped()

        # Set the time stamp to now
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Set the coordinate frame
        goal_pose.header.frame_id = "map"

        # Set the position
        goal_pose.pose.position.x = float(self.x)
        goal_pose.pose.position.y = float(self.y)
        goal_pose.pose.position.z = float(self.z)

        # Set the orientation (quaternion)
        goal_pose.pose.orientation.w = float(self.orientation_w)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0

        # Publish the goal pose
        self.goal_publisher.publish(goal_pose)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()

    try:
        rclpy.spin(node)
    except Exception as e:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
