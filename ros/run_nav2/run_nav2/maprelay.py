import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your message type
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy   


class MapRelay(Node):
    def __init__(self):
        super().__init__("map__relay")

        map_qos = QoSProfile(
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid, "rtabmap/map", self.map_callback, map_qos
        )

        self.map_publisher = self.create_publisher(OccupancyGrid, "map", map_qos)

    def map_callback(self, msg):
        self.map_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    relay = MapRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
