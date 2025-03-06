# This file currently takes in an encoder value from the phidget board and a user input (1,2,3) and sends it to the
# arduino file torque_init.ino on the clearcore through the usb connector.


import serial
import keyboard
import time
from encoder_phidget import phidget_encoder
import threading
import string

# Configuration
PORT = "COM7"  # Change to your serial port
BAUD_RATE = 115200

# Open serial connection
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# setup encoder, channel 0
encoder = phidget_encoder(2)
user_input = ""

def get_user_input():
    """ Thread function to get user input without blocking the continuous write """
    global user_input
    while True:
        user_input = input("")


def read_serial():
    """ Continuously reads data from the serial port. """
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            if line:
                print(f"received:{line}\n")

def write_serial():
    """ Allows user to send serial commands at any time. """
    while True:
        global user_input

        degrees = encoder.get_angle()

        # TODO: currently only writing input and degrees
        command = user_input + ', ' + str(degrees) + '\n'
        # print(f"sent: {command}\n")
        ser.write(command.encode('utf-8'))
        time.sleep(0.1)

# Start reading thread
read_thread = threading.Thread(target=read_serial, daemon=True)
read_thread.start()

input_thread = threading.Thread(target=get_user_input, daemon=True)
input_thread.start()

# Start writing input in the main thread
write_serial()