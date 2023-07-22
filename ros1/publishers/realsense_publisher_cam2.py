#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

#cam 1
#Intel RealSense D435I
#109622072084

#cam 2
#Intel RealSense D435I
#902512070099


class RealSensePublisher_Cam2:
    def __init__(self):
        self.rgb_publisher_cam2 = rospy.Publisher('camera_2/rgb/image_raw', Image, queue_size=20)
        self.depth_publisher_cam2 = rospy.Publisher('camera_2/depth/image_raw', Image, queue_size=20)
        self.bridge = CvBridge()

        # Configure RealSense camera

        self.pipeline2 = rs.pipeline()
        config2 = rs.config()
        config2.enable_device('902512070099')
        config2.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline2.start(config2)

    def publish_images(self):
        # Get frames from RealSense camera
        frames2 = self.pipeline2.wait_for_frames()
        color_frame_cam2 = frames2.get_color_frame()
        depth_frame_cam2 = frames2.get_depth_frame()


        #show the frames
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
        #cv2.imshow('Depth Image', depth_colormap)
        #cv2.waitKey(1)

        # Convert frames to numpy arrays
        img = np.asanyarray(color_frame_cam2.get_data())
        color_image_cam2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        depth_image_cam2 = np.asanyarray(depth_frame_cam2.get_data())

        cv2.imshow('RGB Image', color_image_cam2)
        cv2.waitKey(1)

        # Convert numpy arrays to ROS2 Image messages
        color_msg_cam2 = self.bridge.cv2_to_imgmsg(color_image_cam2, encoding="rgb8")
        depth_msg_cam2 = self.bridge.cv2_to_imgmsg(depth_image_cam2, encoding="passthrough")

        # Publish ROS2 Image messages
        self.rgb_publisher_cam2.publish(color_msg_cam2)
        self.depth_publisher_cam2.publish(depth_msg_cam2)
        rospy.loginfo("published a depth and rgb frame")

    def stop_camera_pipeline(self):
        self.pipeline2.stop()

if __name__ == '__main__':
    rospy.init_node("realsense_publisher_cam2")
    rospy.loginfo("starting realsense publisher")

    realsense_publisher = RealSensePublisher_Cam2()
    while not rospy.is_shutdown():
        realsense_publisher.publish_images()
    cv2.destroyAllWindows()
    realsense_publisher.stop_camera_pipeline()
    rospy.sleep(1.0)
    rospy.loginfo("Shutting down realsense publisher")