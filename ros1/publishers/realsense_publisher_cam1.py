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


class RealSensePublisher_Cam1:
    def __init__(self):
        self.rgb_publisher_cam1 = rospy.Publisher('camera_1/rgb/image_raw', Image, queue_size=20)
        self.depth_publisher_cam1 = rospy.Publisher('camera_1/depth/image_raw', Image, queue_size=20)
        self.bridge = CvBridge()

        # Configure RealSense camera
        self.pipeline1 = rs.pipeline()
        config1 = rs.config()
        config1.enable_device('109622072084')
        config1.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline1.start(config1)

    def publish_images(self):
        # Get frames from RealSense camera
        frames1 = self.pipeline1.wait_for_frames()
        color_frame_cam1 = frames1.get_color_frame()
        depth_frame_cam1 = frames1.get_depth_frame()

        # Convert frames to numpy arrays
        img = np.asanyarray(color_frame_cam1.get_data())
        color_image_cam1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        depth_image_cam1 = np.asanyarray(depth_frame_cam1.get_data())

        cv2.imshow('RGB Image', color_image_cam1)
        cv2.waitKey(1)

        # Convert numpy arrays to ROS2 Image messages
        color_msg_cam1 = self.bridge.cv2_to_imgmsg(color_image_cam1, encoding="rgb8")
        depth_msg_cam1 = self.bridge.cv2_to_imgmsg(depth_image_cam1, encoding="passthrough")

        # Publish ROS2 Image messages
        self.rgb_publisher_cam1.publish(color_msg_cam1)
        self.depth_publisher_cam1.publish(depth_msg_cam1)
        rospy.loginfo("published a depth and rgb frame from camera 1")

    def stop_camera_pipeline(self):
        self.pipeline1.stop()


if __name__ == '__main__':
    rospy.init_node("realsense_publisher_cam1")
    rospy.loginfo("starting realsense publisher")

    realsense_publisher = RealSensePublisher_Cam1()
    while not rospy.is_shutdown():
        realsense_publisher.publish_images()
    cv2.destroyAllWindows()
    realsense_publisher.stop_camera_pipeline()
    rospy.sleep(1.0)
    rospy.loginfo("Shutting down realsense publisher")