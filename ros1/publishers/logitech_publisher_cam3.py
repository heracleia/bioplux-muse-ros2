#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

#"V4L2"
#use the following command in the terminal then lookup the id of the logitech cam. there would be 4 ids each capture stream in a different resolution.
#v4l2-ctl --list-devices
class Logitech_cam3:
    def __init__(self):
        self.rgb_publisher = rospy.Publisher('camera_3/rgb/image_raw', Image, queue_size=20)
        self.cap = None
        self.bridge = CvBridge()

    def publish_images(self):
        # Get frames from RealSense camera
        if self.cap is None:
            self.cap = cv2.VideoCapture(6)
            '''for index in range(0,20): 
                self.cap = cv2.VideoCapture(10)
                if not self.cap.isOpened():
                    continue
                else:
                    print(f"hello I am camera {self.cap.get(cv2.CAP_PROP_BACKEND)} from index {index}")'''
        
        if not self.cap.isOpened():
            rospy.logerr("Could not open video stream")

        while True:
            ret, frame = self.cap.read()

            #rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            cv2.imshow('RGB Image', frame)
            cv2.waitKey(1)

            # Convert numpy arrays to ROS2 Image messages
            color_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")

            # Publish ROS2 Image messages
            self.rgb_publisher.publish(color_msg)
            rospy.loginfo("published a rgb frame")

    def stop_camera_capture(self):
        self.cap.release()


if __name__ == '__main__':
    rospy.init_node("logitech_publisher_cam3")
    rospy.loginfo("starting logitech camera publisher")

    rgb_publisher = Logitech_cam3()
    while not rospy.is_shutdown():
        rgb_publisher.publish_images()
    cv2.destroyAllWindows()
    rgb_publisher.stop_camera_capture()
    rospy.sleep(1.0)
    rospy.loginfo("Shutting down logitech publisher")