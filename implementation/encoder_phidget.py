# general encoder class that receives encoder data from the phidget board

from Phidget22.Devices.Encoder import *
import numpy as np

class phidget_encoder:
    def __init__(self, channel):
        self.encoder = Encoder()
        self.encoder.setChannel(channel)
        self.encoder.openWaitForAttachment(5000)
        self.spool_radius = 1.07  # inches (1 inch spool rad and 0.7 inches for tether width)
        self.angle = 0.0
        self.length = 0.0
        self.init_l = 0.0
        self.init_length()
        # self.spool_width = 1.0  # inches
        # self.angle2length_factor = 1  # inches/degree

    def init_length(self):
        self.init_l = float(input("input initial length (in):"))

    # position to degrees helper function
    def position_to_degrees(self, position):
        # pulses per rev is 2400
        return (position / 2400) * 360

    # get angle from phidget board
    def get_angle(self):
        position = self.encoder.getPosition()
        self.angle = self.position_to_degrees(position)
        return self.angle

    def angle2length(self):
        self.length = self.init_l + (self.angle/360)*(2*np.pi*self.spool_radius)
        return self.length


    # close encoder instance
    def close_encoder(self):
        self.encoder.close()