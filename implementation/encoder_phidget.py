# general encoder class that receives encoder data from the phidget board

from Phidget22.Devices.Encoder import *
class phidget_encoder:
    def __init__(self, channel):
        self.encoder = Encoder()
        self.encoder.setChannel(channel)
        self.encoder.openWaitForAttachment(5000)
        self.angle = 0.0

    # position to degrees helper function
    def position_to_degrees(self, position):
        # pulses per rev is 2400
        return (position / 2400) * 360

    # get angle from phidget board
    def get_angle(self):
        position = self.encoder.getPosition()
        self.angle = self.position_to_degrees(position)
        return self.angle

    # close encoder instance
    def close_encoder(self):
        self.encoder.close()