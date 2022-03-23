import math

import rospy
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import os
from threading import Lock
from enum import Enum


os.environ['MAVLINK20'] = '1'


class OnboardTelemetry:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600, dialect='common')
        self.new_fire_pub = rospy.Publisher("new_fire_bins", Int32MultiArray, queue_size=100)
        self.new_no_fire_pub = rospy.Publisher("new_no_fire_bins", Int32MultiArray, queue_size=100)

    def run(self):
        msg = self.connection.recv_match()
        if msg is None:
            return
        msg = msg.to_dict()
        print(msg)

        if msg['mavpackettype'] == 'TUNNEL':
            payload = msg['payload']
            updated_bins_msg = Int32MultiArray()
            for i in range(int(msg['payload_length']/3)):
                bin_bytes = payload[3*i:3*i+3]
                bin = int.from_bytes(bin_bytes, byteorder='big')
                updated_bins_msg.data.append(bin)

            if msg['payload_type'] == 32768:
                self.new_fire_pub.publish(updated_bins_msg)
            elif msg['payload_type'] == 32769:
                self.new_no_fire_pub.publish(updated_bins_msg)


if __name__ == "__main__":
    rospy.init_node("gcs_telemetry", anonymous=True)
    onboard_telemetry = OnboardTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
