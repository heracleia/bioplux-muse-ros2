#!/usr/bin/env python3
import rospy

from std_msgs.msg import String

import platform
import sys


#download bioplux dependency from https://github.com/pluxbiosignals/python-samples

# connecting the device to computer using bluetooth. The default pin is 123. Some Linux versio may not allow 3 digit pin. In that case the device has to be connected using terminal.

osDic = {"Darwin": "MacOS/plux.so",
         "Linux": "Linux64",
         "Windows": f"Win{platform.architecture()[0][:2]}_{''.join(platform.python_version().split('.')[:2])}"}

print(osDic[platform.system()])
sys.path.append(f"/home/heracleia/Documents/bioplux_depend/PLUX-API-Python3/{osDic[platform.system()]}")

print(sys.path)
import plux
import datetime, time , os , threading
import numpy as np


class bcolors:
    ENDC = '\033[0m'

    ERROR = '\033[1;31m'
    WARNING = '\033[30;41m'

    GOOD =  '\033[1;96m'

    BOLD = '\033[1m'

    MBIENT = '\033[34m'
    BCI = '\033[35m'
    PLUX = '\033[36m'



class MyDevice(plux.MemoryDev):

    def __init__(self, address):
        plux.MemoryDev.__init__(address)
        self.duration = 0
        self.frequency = 0
        self.PluxLoggingFlag = False
        self.exitPluxLoop  = False
        self.publisher = None
        
#this method publishes the raw frames to be written in the subscriber
    def onRawFrame(self, nSeq, data):  
        timestr=datetime.datetime.now().strftime('%H:%M:%S.%f')
        print(f"from onRawFrame {data}")
        msg = String()
        msg.data=f"{timestr},{data[0]},{data[1]}"
        #msg.data = "{0:s},{1:f},{2:f},{3:f}\n".format(timestr, data[0], data[1], data[2])
        if self.publisher is not None:
            self.publisher.publish(msg)
        if nSeq % 1000 == 0:
           print(bcolors.PLUX + "--Plux-- ", nSeq, data  , bcolors.ENDC)# Print out a data frame every 1000 frames)
        if self.exitPluxLoop:
            print(bcolors.GOOD + "--Plux-- Exiting Plux Loop" + bcolors.ENDC)
            return True
        return False


    def onEvent(self, event):
        if type(event) == plux.Event.DigInUpdate:
            print(bcolors.PLUX + '--Plux-- Digital input event - Clock source:', event.timestamp.source, \
                  ' Clock value:', event.timestamp.value, ' New input state:', event.state, bcolors.ENDC)
        elif type(event) == plux.Event.SchedChange:
            print('--Plux-- Schedule change event - Action:', event.action, \
                  ' Schedule start time:', event.schedStartTime)
        elif type(event) == plux.Event.Sync:
            print('--Plux-- Sync event:')
            for tstamp in event.timestamps:
                print(' Clock source:', tstamp.source, ' Clock value:', tstamp.value)
        elif type(event) == plux.Event.Disconnect:
            print('--Plux-- Disconnect event - Reason:', event.reason)
            return True # Exit loop() after receiving a disconnect event
        return False

    def onInterrupt(self, param):
        print('--Plux-- Interrupt:', param)
        return False

    def onTimeout(self):
        print('--Plux-- Timeout')
        return False

    def onSessionRawFrame(self, nSeq, data):
        print >>self.f, nSeq, self.lastDigState,
        for val in data:
            print >>self.f, val,
        print >>self.f
        if nSeq % 1000 == 0:
            print('--Plux-- Session:', nSeq, data)
        return False

    def onSessionEvent(self, event):
        if type(event) == plux.Event.DigInUpdate:
            print('--Plux-- Session digital input event - Clock source:', event.timestamp.source, \
                  ' Clock value:', event.timestamp.value, ' New input state:', event.state)
            self.lastDigState = 1 if event.state else 0
        elif type(event) == plux.Event.Sync:
            print('--Plux-- Session sync event:')
            for tstamp in event.timestamps:
                print(' Clock source:', tstamp.source, ' Clock value:', tstamp.value)
        return False

    def return_logged_data(self):
        global data2
        # data3 = data2
        # data2=[]
        return data2



class PluxPublisher():
    def __init__(self):
        self.publisher = rospy.Publisher("/plux_topic", String, queue_size=256)
        timer_period = 1  # seconds
        self.device = None
        #self.plux_file = None

    def start_streaming(self):

        print("## Connecting biosignalsplux sensors ##")
        self.connect_sensor()

    def connect_sensor(self):
        """
        Handler Function to connect to the biosignalsplux sensor and create a file to write the data
        :return: None
        """
        try:
            address = "00:07:80:46:E5:C0" # MAC address of device
            print(f"trying to connect to device {address}")
            self.device = MyDevice(address)
            print("No error. move on\n")  
            props = self.device.getProperties()  # get and print device properties
            if props is  not None:
                print(bcolors.GOOD + "--Plux-- Connection is established" + bcolors.ENDC)
            self.device.publisher=self.publisher
            self.device.PluxLoggingFlag = False
            self.device.exitPluxLoop = False
            
            # port1: ECG , port2: GSR , port4: EMG
            src_a = plux.Source()
            src_a.port = 1

            src_b = plux.Source()
            src_b.port = 2

            #src_c = plux.Source()
            #src_c.port = 4

            frequency=1000
            self.device.start(frequency, (src_a, src_b))
            #self.device.start(frequency, (src_a, src_b, src_c))

            print(bcolors.GOOD + "--Plux-- Started Streaming" + bcolors.ENDC)

            self.device.loop()  # calls device.onRawFrame until it returns True

            self.device.stop()
            print(bcolors.GOOD + "--Plux-- Stopped Plux Connection" + bcolors.ENDC)

        except Exception as e:
            print(bcolors.ERROR +  "--Plux-- Error: Inside connect_sensor()" + bcolors.ENDC)
            print(bcolors.ERROR + str(e) + bcolors.ENDC)
            if (self.device):
                self.device.close()

    def close_sensor(self):
        """
         Handler function to disconnect and close the sensor
         :return: None
        """

        # Set variable to stop recording from biosignalsplux

        self.device.PluxLoggingFlag = False
        self.device.exitPluxLoop = True

        if self.device is not None:
            rospy.sleep(0.2)
            self.device.close()
            print(bcolors.GOOD + "--Plux-- Closed Plux Connection" + bcolors.ENDC)
        else:
            print(bcolors.ERROR + "--Plux-- Error: Dev is not initialized - Line 233" + bcolors.ENDC)
    
    

if __name__ == '__main__':
    rospy.init_node("bioplux_publisher")
    rospy.loginfo("starting bioplux publisher")

    string_publisher = PluxPublisher()
    string_publisher.start_streaming()
    string_publisher.close_sensor()

    rospy.sleep(1.0)
    rospy.loginfo("Shutting down bioplux publisher")