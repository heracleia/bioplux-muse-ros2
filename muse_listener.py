#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import rosbag2_py
from muse_interfaces.msg import TargetMessage
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import csv

class Subscriber(Node):

    def __init__(self, file_name):
        super().__init__('muse_subscriber')
        print(file_name)
        self.file_name = file_name
        self.csv_file = open(self.file_name, "w")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "TP9", "AF7", "AF8", "TP10"])
        #self.writer = rosbag2_py.SequentialWriter()

        # storage_options = rosbag2_py._storage.StorageOptions(
        #     uri='my_bag',
        #     storage_id='sqlite3')
        # converter_options = rosbag2_py._storage.ConverterOptions('', '')
        # self.writer.open(storage_options, converter_options)

        # topic_info = rosbag2_py._storage.TopicMetadata(
        #     name='eeg',
        #     type='std_msgs/msg/String',
        #     serialization_format='cdr')
        # self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            TargetMessage,
            'eeg',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        print(msg)
        str_msg = str(msg)
        # x = str_msg.split(", ")
        # print(x) 

        self.csv_writer.writerow([msg.timestamp, msg.tp9, msg.af7, msg.af8, msg.tp10])
        # self.writer.write(
        #     'eeg',
        #     serialize_message(msg),.
        #     self.get_clock().now().nanoseconds)

    def __del__(self):
        self.csv_file.close()

def main():
    PATH = sys.argv[1:]
    rclpy.init(args=PATH)
    file_name = "output.csv"
    subscriber = Subscriber(file_name)
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    del subscriber
    subscriber.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()