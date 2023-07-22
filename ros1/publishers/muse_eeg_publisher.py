#!/usr/bin/env python3
import rospy
from muse_helper import Muse


if __name__ == "__main__":
    rospy.init_node("muse_eeg_publisher")
    rospy.Rate(256)
    muse_sensor = Muse()
    while not rospy.is_shutdown():
        muse_sensor.publish_eeg_messages()
