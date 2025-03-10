# testing encoder to length conversion


import serial
import keyboard
import time
from encoder_phidget import phidget_encoder
import threading
import string

# setup encoder, channel 0
encoder = phidget_encoder(0)

while True:
    degrees = encoder.get_angle()
    length = encoder.angle2length()
    print(f"deg: {degrees}, L: {length}")