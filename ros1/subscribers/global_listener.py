#!/usr/bin/env python3
import sys
from time import gmtime, strftime
import rospy
import rosbag
from dare_sensors.msg import EEGMessage
from std_msgs.msg import String
from sensor_msgs.msg import Image
import csv
import cv2
import numpy as np
import os

class Subscriber:

    def __init__(self, directory_path):
        
        print(directory_path)
        self.directory_path = directory_path
        self.data_source="EEG"
        #initialize csv for writing eeg data
        
    
        self.muse_csv_file = open(self.directory_path+"/raw_eeg.csv", "w")
        self.muse_csv_writer = csv.writer(self.muse_csv_file)
        self.muse_csv_writer.writerow(["timestamp", "TP9", "AF7", "AF8", "TP10"])

        #initialize csv for writing eeg data
        self.plux_csv_file = open(self.directory_path+"/plux.csv", "w")
        self.plux_csv_writer = csv.writer(self.plux_csv_file)
        self.plux_csv_writer.writerow(["timestamp", "ECG", "GSR"])


        #initialize the video writers
        self.rgb_video_filename1 = self.directory_path + "/video_1.avi"
        self.rgb_codec1 = cv2.VideoWriter_fourcc(*'XVID')
        self.rgb_fps1 = 30.0
        self.rgb_writer1 = None

        self.rgb_video_filename2 = self.directory_path + "/video_2.avi"
        self.rgb_codec2 = cv2.VideoWriter_fourcc(*'XVID')
        self.rgb_fps2 = 30.0
        self.rgb_writer2 = None

        self.rgb_video_filename3 = self.directory_path + "/video_3.avi"
        self.rgb_codec3 = cv2.VideoWriter_fourcc(*'XVID')
        self.rgb_fps3 = 30.0
        self.rgb_writer3 = None

        #rosbag to store the depth. conversion to xvid format might result in loss of infromation. 
        self.depth_bag = rosbag.Bag(self.directory_path + "/depth_data.bag", 'w')

        #create the subscribers
        #subscribers for realsense cam1
        self.depth_sub_cam1 = rospy.Subscriber('camera_1/depth/image_raw', Image, self.depth_callback_cam1)
        self.rgb_sub_cam1 = rospy.Subscriber('camera_1/rgb/image_raw', Image, self.rgb_callback_cam1)

        #subscribers for realsense cam2 
        self.depth_sub_cam2 = rospy.Subscriber('camera_2/depth/image_raw', Image, self.depth_callback_cam2)
        self.rgb_sub_cam2 = rospy.Subscriber('camera_2/rgb/image_raw', Image, self.rgb_callback_cam2)

        #subscribers for logitech cam3
        self.rgb_sub_cam3 = rospy.Subscriber('camera_3/rgb/image_raw', Image, self.rgb_callback_cam3) 

        self.muse_subscriber = rospy.Subscriber("/eeg_stream", EEGMessage, self.muse_listener_callback)
        
        self.plux_subscriber = rospy.Subscriber("/plux_topic", String, self.plux_listener_callback)
    
    
    def depth_callback_cam1(self, msg):
        self.depth_bag.write('camera_1/depth/image_raw', msg)

    def depth_callback_cam2(self, msg):
        self.depth_bag.write('camera_2/depth/image_raw', msg)

    def rgb_callback_cam1(self, msg):
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.rgb_writer1 is None:
            self.rgb_writer1 = cv2.VideoWriter(self.rgb_video_filename1, self.rgb_codec1, self.rgb_fps1,(rgb_image.shape[1], rgb_image.shape[0]))
        self.rgb_writer1.write(rgb_image)
    
    def rgb_callback_cam2(self, msg):
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.rgb_writer2 is None:
            self.rgb_writer2 = cv2.VideoWriter(self.rgb_video_filename2, self.rgb_codec2, self.rgb_fps2,(rgb_image.shape[1], rgb_image.shape[0]))
        self.rgb_writer2.write(rgb_image)

    def rgb_callback_cam3(self, msg):
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.rgb_writer3 is None:
            self.rgb_writer3 = cv2.VideoWriter(self.rgb_video_filename3, self.rgb_codec3, self.rgb_fps3, (rgb_image.shape[1], rgb_image.shape[0]))
        self.rgb_writer3.write(rgb_image)

    def muse_listener_callback(self, msg):
        rospy.loginfo('I heard: "%s"' % msg)
        print(msg)
        self.muse_csv_writer.writerow([msg.timestamp, msg.tp9, msg.af7, msg.af8, msg.tp10])
        self.muse_csv_file.flush()


    def plux_listener_callback(self, msg):
        rospy.loginfo('Received message from bioplux: "%s"' % msg.data)
        msg_tkns = str(msg.data).split(",")
        self.plux_csv_writer.writerow([msg_tkns[0], msg_tkns[1], msg_tkns[2]])
        self.plux_csv_file.flush()

    def close_resources(self):
        if self.muse_csv_file is not None:
            self.muse_csv_file.close()
        if self.plux_csv_file is not None:
            self.plux_csv_file.close()
        self.depth_bag.close()
        if self.rgb_writer1 is not None:
            self.rgb_writer1.release()
        if self.rgb_writer2 is not None:
            self.rgb_writer2.release()
        if self.rgb_writer3 is not None:
            self.rgb_writer3.release()

    def __del__(self):
        pass

def main():
    PATH = sys.argv[1]
    print(PATH)
    rospy.init_node("global_sensor_listener", anonymous=True)
    subscriber = Subscriber(PATH)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    subscriber.close_resources()
    



if __name__ == '__main__':
    main()