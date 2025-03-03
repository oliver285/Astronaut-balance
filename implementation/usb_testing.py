import serial
import keyboard
import time
from Phidget22.Devices.Encoder import *
import threading
import string

# Configuration
PORT = "COM7"  # Change to your serial port
BAUD_RATE = 115200

# Open serial connection
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

def encoder_setup():
    # Define the encoder object
    encoder = Encoder()
    # Set the encoder channel (change this to match your setup)
    encoder.setChannel(0)
    # Open the encoder
    encoder.openWaitForAttachment(5000)  # Wait up to 5 seconds for attachment
    return encoder

encoder = encoder_setup()

def position_to_degrees(position, pulses_per_revolution):
    return (position / pulses_per_revolution) * 360

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

        # if keyboard.is_pressed("1"):  # if key 'q' is pressed
        #     user_input = "1"
        # elif keyboard.is_pressed("2"):  # if key 'q' is pressed
        #     user_input = "2"
        # elif keyboard.is_pressed("3"):  # if key 'q' is pressed
        #     user_input = "3"

        position = encoder.getPosition()
        # print(position)
        # Convert counts to degrees (assuming 600 pulses per revolution)
        pulses_per_revolution = 2400  # encoder counts value
        degrees = round(position_to_degrees(position, pulses_per_revolution), 3)
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


# from Phidget22.Devices.Encoder import *
# import threading
# import serial
# import time
#
# # Configuration
# PORT = "COM7"  # Change to your serial port
# BAUD_RATE = 115200
#
# # Open serial connection
# ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
#
#
# def encoder_setup():
#     # Define the encoder object
#     encoder = Encoder()
#     # Set the encoder channel (change this to match your setup)
#     encoder.setChannel(0)
#     # Open the encoder
#     encoder.openWaitForAttachment(5000)  # Wait up to 5 seconds for attachment
#     return encoder
#
#
# encoder = encoder_setup()
#
#
# def position_to_degrees(position, pulses_per_revolution):
#     return (position / pulses_per_revolution) * 360
#
#
# # Global variable to store the latest user input
# user_command = ""
#
#
# def read_serial():
#     """ Continuously reads data from the serial port. """
#     while True:
#         if ser.in_waiting > 0:
#             line = ser.readline().decode('utf-8', errors='ignore').rstrip()
#             if line:
#                 print(line)
#
#
# def get_user_input():
#     """ Thread function to get user input without blocking the continuous write """
#     global user_command
#     while True:
#         user_command = input("")
#
#
# def write_serial():
#     """ Continuously writes to serial port, updating with the latest user input """
#     global user_command
#     while True:
#         position = encoder.getPosition()
#         degrees = position_to_degrees(position, 2400)  # 2400 pulses per revolution
#
#         command = user_command + ', ' + str(degrees) + '\n' if user_command else str(degrees) + '\n'
#
#         if user_command:  # Only print when there's a command to avoid console spam
#             print(command.rstrip())
#
#         ser.write(command.encode('utf-8'))
#         time.sleep(0.1)  # Adjust the frequency as needed
#
#
# # Start reading thread
# read_thread = threading.Thread(target=read_serial, daemon=True)
# read_thread.start()
#
# # Start user input thread
# input_thread = threading.Thread(target=get_user_input, daemon=True)
# input_thread.start()
#
# # Start continuous writing in the main thread
# write_serial()
