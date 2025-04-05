
import pandas as pd
import matplotlib.pyplot as plt




# Load CSV data
df = pd.read_csv('tether_data_20250404_9.csv')  # Replace with your actual CSV filename

# Convert UTC Time to datetime (optional for nicer plotting)
df['UTC Time'] = pd.to_datetime(df['UTC Time'], errors='coerce')

# Example 1: Plot Tether Lengths Over Time
plt.figure(figsize=(10, 6))
plt.plot(df['UTC Time'], df['Tether 1 Length'], label='Tether 1 Length')
plt.plot(df['UTC Time'], df['Tether 2 Length'], label='Tether 2 Length')
plt.plot(df['UTC Time'], df['Tether 3 Length'], label='Tether 3 Length')
plt.xlabel('Time')
plt.ylabel('Tether Length (m)')
plt.title('Tether Lengths Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Example 2: Tether Forces
plt.figure(figsize=(10, 6))
plt.plot(df['UTC Time'], df['Tether 1 Force'], label='Tether 1 Force')
plt.plot(df['UTC Time'], df['Tether 2 Force'], label='Tether 2 Force')
plt.plot(df['UTC Time'], df['Tether 3 Force'], label='Tether 3 Force')
plt.xlabel('Time')
plt.ylabel('Force (N)')
plt.title('Tether Forces Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Example 3: Apex Position (X, Y, Z)
plt.figure(figsize=(10, 6))
plt.plot(df['UTC Time'], df['X Apex'], label='X Apex')
plt.plot(df['UTC Time'], df['Y Apex'], label='Y Apex')
plt.plot(df['UTC Time'], df['Z Apex'], label='Z Apex')
plt.xlabel('Time')
plt.ylabel('Position (m)')
plt.title('Apex Position Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
