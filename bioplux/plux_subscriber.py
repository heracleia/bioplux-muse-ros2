import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PluxSubscriber(Node):

    def __init__(self):
        super().__init__('plux_subscriber')
        self.subscription = self.create_subscription(
            String,
            'plux_topic',
            self.message_callback,
            10)
        self.subscription  # prevent unused variable warning

    def message_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    string_subscriber = PluxSubscriber()

    rclpy.spin(string_subscriber)

    string_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
