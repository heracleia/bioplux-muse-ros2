#!/usr/bin/env python3
from pylsl import StreamInlet, resolve_stream
import rospy
from muse_helper import Muse



class MuseStream:
    def __init__(self):
        # rospy.init_node("museStreamer", anonymous=True)
        r = rospy.Rate(256)  # 10hz
        rospy.loginfo("Starting EEG Stream")

    if __name__ == "__main__":
        rospy.init_node("muse_eeg_stream")
        muse_sensor = Muse()
        muse_sensor.start_eeg_stream()
