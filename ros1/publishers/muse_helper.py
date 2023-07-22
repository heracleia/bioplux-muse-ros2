from pylsl import StreamInlet, resolve_stream
import rospy
from dare_sensors.msg import EEGMessage
import subprocess
from pylsl import StreamInlet, resolve_byprop
from constants import LSL_SCAN_TIMEOUT, LSL_EEG_CHUNK, LSL_PPG_CHUNK, LSL_ACC_CHUNK, LSL_GYRO_CHUNK
#from time import time, strftime, gmtime
# from typing import Union, List, Optional
# from pathlib import Path
#from sklearn.linear_model import LinearRegression
# import numpy as np
# import pandas as pd

class Muse:
    def __init__(self):
        rospy.Rate(256)
        self.muse_publisher = rospy.Publisher(
            "eeg_stream", EEGMessage, queue_size=256
        )
        self.data_source="EEG"
           
    def start_eeg_stream(self):
        print("Trying to start EEG stream...")
        subprocess.run(["muselsl", "stream"])


    def publish_eeg_messages(self):
        self.streams = resolve_stream("type", "EEG")
        chunk_length = LSL_EEG_CHUNK

        print("Looking for a %s stream..." % (self.data_source))
        streams = resolve_byprop('type', self.data_source, timeout=LSL_SCAN_TIMEOUT)

        if len(streams) == 0:
            print("Can't find %s stream." % (self.data_source))
            return

        print("Started acquiring data.")
        inlet = StreamInlet(streams[0], max_chunklen=chunk_length)

        while True:
            try:
                data, timestamp = inlet.pull_chunk(
                    timeout=1.0, max_samples=chunk_length)
                
                for d, t in zip(data, timestamp):
                    msg = EEGMessage()
                    msg.tp9 = d[0]
                    msg.af7 = d[1]
                    msg.af8 = d[2]
                    msg.tp10 = d[3]
                    msg.timestamp = t
                    rospy.loginfo(msg)
                    self.muse_publisher.publish(msg)
    
            except KeyboardInterrupt:
                break

    def convert_to_neurological(self):
        pass
