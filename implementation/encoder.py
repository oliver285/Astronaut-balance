from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
import time

# Define the encoder object
encoder = Encoder()
# Set the encoder channel (change this to match your setup)
encoder.setChannel(0)
# Open the encoder
encoder.openWaitForAttachment(5000)  # Wait up to 5 seconds for attachment

# Function to calculate degrees from position
def position_to_degrees(position, pulses_per_revolution):
    return (position / pulses_per_revolution) * 360

# Main loop to read and print encoder position
try:
    print("Reading encoder position. Press key to stop.")
    while True:
        # Get the current position in pulses
        position = encoder.getPosition()
        # print(position)
        # Convert counts to degrees (assuming 600 pulses per revolution)
        pulses_per_revolution = 2400  # encoder counts value
        degrees = position_to_degrees(position, pulses_per_revolution)
        
        # Print the position in degrees
        print(f"Position: {degrees:.2f} degrees")
        
        # Wait for a short time before reading again
        # time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping encoder reading.")

finally:
    # Close the encoder
    encoder.close()
