

import pandas as pd
import matplotlib.pyplot as plt




# Load CSV data
df = pd.read_csv('tether_data_20250404_9.csv')  # Replace with your actual CSV filename

# Convert UTC Time to datetime (optional for nicer plotting)
# df['Microseconds Since Start'] = pd.to_datetime(df['Microseconds Since Start'], errors='coerce')

# Example 1: Plot Tether Lengths Over Time
plt.figure(figsize=(10, 6))
plt.plot(df['Microseconds Since Start'], df['Tether 1 Length'], label='Tether 1 Length')
plt.plot(df['Microseconds Since Start'], df['Tether 2 Length'], label='Tether 2 Length')
plt.plot(df['Microseconds Since Start'], df['Tether 3 Length'], label='Tether 3 Length')
plt.xlabel('Time (ms)')
plt.ylabel('Tether Length (in)')
plt.title('Tether Lengths Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Example 2: Tether Forces
plt.figure(figsize=(10, 6))
plt.plot(df['Microseconds Since Start'], df['Tether 1 Force'], label='Tether 1 Force')
plt.plot(df['Microseconds Since Start'], df['Tether 2 Force'], label='Tether 2 Force')
plt.plot(df['Microseconds Since Start'], df['Tether 3 Force'], label='Tether 3 Force')
plt.xlabel('Time (ms)')
plt.ylabel('Force (N)')
plt.title('Tether Forces Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Example 3: Apex Position (X, Y, Z)
plt.figure(figsize=(10, 6))
plt.plot(df['Microseconds Since Start'], df['X Apex'], label='X Apex')
plt.plot(df['Microseconds Since Start'], df['Y Apex'], label='Y Apex')
plt.plot(df['Microseconds Since Start'], df['Z Apex'], label='Z Apex')
plt.xlabel('Time (ms)')
plt.ylabel('Position (in)')
plt.title('Apex Position Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Example 4: Force Error (X, Y, Z)
plt.figure(figsize=(10, 6))
plt.plot(df['Microseconds Since Start'], df['Force Error (lbf)'], label='Force Error (lbf)')
plt.xlabel('Time (ms)')
plt.ylabel('Force Error (lbf)')
plt.title('Force Error (lbf)')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Example 5: Angle Error (X, Y, Z)
plt.figure(figsize=(10, 6))
plt.plot(df['Microseconds Since Start'], df['Angle Error (deg)'], label='Angle Error (deg)')
plt.xlabel('Time (ms)')
plt.ylabel('Angle Error (deg)')
plt.title('Angle Error (deg)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
