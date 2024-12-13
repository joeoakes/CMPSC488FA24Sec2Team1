import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your message type
from nav_msgs.msg import Odometry


class TopicRelay(Node):
    def __init__(self):
        super().__init__("topic_relay")
        self.subscription = self.create_subscription(
            Odometry, "rtabmap/odom", self.listener_callback, 10
        )

        self.publisher = self.create_publisher(Odometry, "odom", 10)

    def listener_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    relay = TopicRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
