import math

import rospy
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import os
from threading import Lock
from enum import Enum


os.environ['MAVLINK20'] = '1'


class PayloadTunnelType(Enum):
    FireBins = 32768
    NonFireBins = 32769


class OnboardTelemetry:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600 ,dialect='common')

        self.new_fire_bins = []
        self.new_no_fire_bins = []
        self.new_bins_mutex = Lock()

        rospy.Subscriber("new_fire_bins", Int32MultiArray, self.new_fire_bins_callback)
        rospy.Subscriber("new_no_fire_bins", Int32MultiArray, self.new_no_fire_bins_callback)

    def new_fire_bins_callback(self, data: Int32MultiArray):
        with self.new_bins_mutex:
            self.new_fire_bins.extend(data.data)

    def new_no_fire_bins_callback(self, data: Int32MultiArray):
        with self.new_bins_mutex:
            self.new_no_fire_bins.extend(data.data)

    def send_map_update(self):
        updates_to_send = None
        sending_fire_bins = None
        max_bins_to_send = math.floor(128/3) # Since max payload is 128 bytes and each bin represented by 3 bytes
        with self.new_bins_mutex:
            if len(self.new_fire_bins) > 0:
                updates_to_send = self.new_fire_bins[:max_bins_to_send]
                self.new_fire_bins = self.new_fire_bins[max_bins_to_send:]
                sending_fire_bins = True
            elif len(self.new_no_fire_bins) > 0:
                updates_to_send = self.new_no_fire_bins[:max_bins_to_send]
                self.new_no_fire_bins = self.new_no_fire_bins[max_bins_to_send:]
                sending_fire_bins = False
            else:
                return

        payload = bytearray()
        for update in updates_to_send:
            payload.extend(update.to_bytes(3, byteorder='big'))

        payload_length = len(payload)
        if len(payload) < 128:
            payload.extend(bytes([0]*(128-len(payload))))  # Pad payload so it has 128 bytes

        if sending_fire_bins:
            self.connection.mav.tunnel_send(0, 0, PayloadTunnelType.FireBins.value, payload_length, payload)
        else:
            self.connection.mav.tunnel_send(0, 0, PayloadTunnelType.NonFireBins.value, payload_length, payload)

        # Tunnel message is 145 bytes. Sleep by this much to not overwhelm the serial baud rate
        rospy.sleep(145.0/1152.0)

    def run(self):
        self.send_map_update()


if __name__ == "__main__":
    rospy.init_node("onboard_telemetry", anonymous=True)
    onboard_telemetry = OnboardTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
