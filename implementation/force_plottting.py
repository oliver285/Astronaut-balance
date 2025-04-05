import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

# Serial port configuration
SERIAL_PORT = "COM7"  # Change this to match your system
BAUD_RATE = 115200

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# Data storage
time_data = []
force1_data = []
force2_data = []
force3_data = []

latest_data = None  # Store the most recent valid data

# Figure and axes setup
fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
axs[0].set_ylabel("Force 1 (lb)")
axs[1].set_ylabel("Force 2 (lb)")
axs[2].set_ylabel("Force 3 (lb)")
axs[2].set_xlabel("Time (s)")

def serial_reader():
    """Continuously read from the serial port and store the latest valid line."""
    global latest_data
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:  # Only update if the line is not empty
                latest_data = line
        except Exception as e:
            print(f"Serial read error: {e}")

# Start the serial reading thread
serial_thread = threading.Thread(target=serial_reader, daemon=True)
serial_thread.start()

def update_plot(frame):
    """Update the plot using the most recent valid serial data."""
    global time_data, force1_data, force2_data, force3_data, latest_data
    # print(latest_data)
    if latest_data:  # Only process if there's new data
        values = latest_data.split(",")
        if len(values) == 4:  # Ensure correct format
            try:
                t, f1, f2, f3 = map(float, values)  # Convert to floats

                # Append to lists
                time_data.append(t)
                force1_data.append(f1)
                force2_data.append(f2)
                force3_data.append(f3)


                # Keep only the last 100 points for performance
                time_data = time_data[-100:]
                force1_data = force1_data[-100:]
                force2_data = force2_data[-100:]
                force3_data = force3_data[-100:]


                # Clear and replot
                axs[0].cla()
                axs[1].cla()
                axs[2].cla()

                axs[0].plot(time_data, force1_data, 'r', label="Force 1")
                axs[1].plot(time_data, force2_data, 'g', label="Force 2")
                axs[2].plot(time_data, force3_data, 'b', label="Force 3")

                axs[0].legend()
                axs[1].legend()
                axs[2].legend()

                axs[0].set_ylabel("Force 1 (lb)")
                axs[1].set_ylabel("Force 2 (lb)")
                axs[2].set_ylabel("Force 3 (lb)")
                axs[2].set_xlabel("Time (s)")

            except ValueError:
                print(f"Invalid data received: {latest_data}")

def send_commands():
    """Function to send commands over serial from user input."""
    while True:
        user_input = input("")
        if user_input.lower() == "exit":
            print("Exiting command input thread.")
            break
        ser.write((user_input + "\n").encode())

# Start a separate thread for user input
command_thread = threading.Thread(target=send_commands, daemon=True)
command_thread.start()

# Animation for real-time updating
ani = animation.FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)

plt.show()

# Close the serial connection when the script stops
ser.close()









# this run with an output of the load cell and commands torque for debugging

# import serial
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import threading
# import time
#
# # Serial port configuration
# SERIAL_PORT = "COM7"  # Change this to match your system
# BAUD_RATE = 115200
#
# # Initialize serial connection
# ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
#
# # Data storage
# time_data = []
# force1_data = []
# force2_data = []
# force3_data = []
# T1_data = []
# T2_data = []
# T3_data = []
#
# latest_data = None  # Store the most recent valid data
#
# # Figure and axes setup
# fig, axs = plt.subplots(3, 2, figsize=(8, 6), sharex=True)
# axs[0,0].set_ylabel("Force 1 (lb)")
# axs[1,0].set_ylabel("Force 2 (lb)")
# axs[2,0].set_ylabel("Force 3 (lb)")
# axs[2,0].set_xlabel("Time (s)")
# axs[0,1].set_ylabel("torque 1")
# axs[1,1].set_ylabel("torque 2")
# axs[2,1].set_ylabel("torque 3")
# axs[2,1].set_xlabel("Time (s)")
#
# def serial_reader():
#     """Continuously read from the serial port and store the latest valid line."""
#     global latest_data
#     while True:
#         try:
#             line = ser.readline().decode('utf-8').strip()
#             if line:  # Only update if the line is not empty
#                 latest_data = line
#         except Exception as e:
#             print(f"Serial read error: {e}")
#
# # Start the serial reading thread
# serial_thread = threading.Thread(target=serial_reader, daemon=True)
# serial_thread.start()
#
# def update_plot(frame):
#     """Update the plot using the most recent valid serial data."""
#     global time_data, force1_data, force2_data, force3_data, T1_data, T2_data, T3_data, latest_data
#     # print(latest_data)
#     if latest_data:  # Only process if there's new data
#         values = latest_data.split(",")
#         if len(values) == 7:  # Ensure correct format
#             try:
#                 t, f1, f2, f3, T1, T2, T3 = map(float, values)  # Convert to floats
#                 # Append to lists
#                 time_data.append(t)
#                 force1_data.append(f1)
#                 force2_data.append(f2)
#                 force3_data.append(f3)
#                 T1_data.append(T1)
#                 T2_data.append(T2)
#                 T3_data.append(T3)
#
#                 # Keep only the last 100 points for performance
#                 time_data = time_data[-100:]
#                 force1_data = force1_data[-100:]
#                 force2_data = force2_data[-100:]
#                 force3_data = force3_data[-100:]
#                 T1_data = T1_data[-100:]
#                 T2_data = T2_data[-100:]
#                 T3_data = T3_data[-100:]
#
#                 # Clear and replot
#                 axs[0,0].cla()
#                 axs[1,0].cla()
#                 axs[2,0].cla()
#                 axs[0,1].cla()
#                 axs[1,1].cla()
#                 axs[2,1].cla()
#
#                 axs[0,0].plot(time_data, force1_data, 'r', label="Force 1")
#                 axs[1,0].plot(time_data, force2_data, 'g', label="Force 2")
#                 axs[2,0].plot(time_data, force3_data, 'b', label="Force 3")
#                 axs[0,1].plot(time_data, T1_data, 'r', label="torque 1")
#                 axs[1,1].plot(time_data, T2_data, 'g', label="torque 2")
#                 axs[2,1].plot(time_data, T3_data, 'b', label="torque 3")
#
#                 axs[0, 0].set_ylabel("Force 1 (lb)")
#                 axs[1, 0].set_ylabel("Force 2 (lb)")
#                 axs[2, 0].set_ylabel("Force 3 (lb)")
#                 axs[2, 0].set_xlabel("Time (s)")
#                 axs[0, 1].set_ylabel("torque 1")
#                 axs[1, 1].set_ylabel("torque 2")
#                 axs[2, 1].set_ylabel("torque 3")
#                 axs[2, 1].set_xlabel("Time (s)")
#
#             except ValueError:
#                 print(f"Invalid data received: {latest_data}")
#
# def send_commands():
#     """Function to send commands over serial from user input."""
#     while True:
#         user_input = input("")
#         if user_input.lower() == "exit":
#             print("Exiting command input thread.")
#             break
#         ser.write((user_input + "\n").encode())
#
# # Start a separate thread for user input
# command_thread = threading.Thread(target=send_commands, daemon=True)
# command_thread.start()
#
# # Animation for real-time updating
# ani = animation.FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)
#
# plt.show()
#
# # Close the serial connection when the script stops
# ser.close()

