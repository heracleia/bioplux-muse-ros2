#!/usr/bin/env python3

from pylsl import StreamInlet, resolve_stream
#from std_msgs.msg import Float32MultiArray
#import rospy
#from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from muse_interfaces.msg import TargetMessage

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.publisher_ = self.create_publisher(TargetMessage, 'eeg', 10)
        
    def stream_muse_data(self):
        # first resolve an EEG stream on the lab network
        print("looking for an EEG stream...")
        streams = resolve_stream('type', 'EEG')
        # create a new inlet to read from the stream
        inlet = StreamInlet(streams[0])
        while rclpy.ok():
            sample,timestamp = inlet.pull_sample()
            msg = TargetMessage()
            #msg.tp9 = sample[0]
            msg.tp9 = sample[0]
            msg.af7 = sample[1]
            msg.af8 = sample[2]
            msg.tp10 = sample[3]
            msg.timestamp = timestamp  
            message = str(sample)+" "+str(timestamp)
            print(msg)
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    node.stream_muse_data()
    rclpy.shutdown()

if __name__ == '__main__':
    main()