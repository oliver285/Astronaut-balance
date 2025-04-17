import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data
df = pd.read_csv(r"file_path_here")  # Replace with your actual CSV filename

# Convert UTC Time to datetime (optional for nicer plotting)
# df['Microseconds Since Start'] = pd.to_datetime(df['Microseconds Since Start'], errors='coerce')

# Example 1: Plot Tether Lengths Over Time
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['Tether1Length'], label='Tether 1 Length')
plt.plot(df['ResponseTime']/1000, df['Tether2Length'], label='Tether 2 Length')
plt.plot(df['ResponseTime']/1000, df['Tether3Length'], label='Tether 3 Length')
plt.xlabel('Time (s)')
plt.ylabel('Tether Length (in)')
plt.title('Tether Lengths Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Example 2: Tether Forces
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['Tether1Force'], label='Tether 1 Force')
plt.plot(df['ResponseTime']/1000, df['Loadcell1Force'], label='Loadcell 1 Force')
plt.xlabel('Time (s)')
plt.ylabel('Force (lbf)')
plt.title('Tether 1 Force Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Example 2: Tether Forces
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['Tether2Force'], label='Tether 2 Force')
plt.plot(df['ResponseTime']/1000, df['Loadcell2Force'], label='Loadcell 2 Force')
plt.xlabel('Time (s)')
plt.ylabel('Force (lbf)')
plt.title('Tether 2 Force Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Example 2: Tether Forces
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['Tether3Force'], label='Tether 3 Force')
plt.plot(df['ResponseTime']/1000, df['Loadcell3Force'], label='Loadcell 3 Force')
plt.xlabel('Time (s)')
plt.ylabel('Force (lbf)')
plt.title('Tether 3 Force Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Example 3: Apex Position (X, Y, Z)
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['XApex']*12, label='X Apex')
plt.plot(df['ResponseTime']/1000, df['YApex']*12, label='Y Apex')
plt.plot(df['ResponseTime']/1000, df['ZApex']*12, label='Z Apex')
plt.xlabel('Time (s)')
plt.ylabel('Position (in)')
plt.title('Assumed User CG Position Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Example 4: Force Error (X, Y, Z) with requirement line
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['ForceError_lbf_'], label='System')
# Add horizontal dashed red line at 5 lbs
plt.axhline(y=5, color='r', linestyle='--', label='Requirement (5 lbf)')
plt.xlabel('Time (s)')
plt.ylabel('Force Magnitude Error (lbf)')
plt.title('Downward Force Magnitude Error')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Example 5: Angle Error (X, Y, Z) with requirement line
plt.figure(figsize=(10, 6))
plt.plot(df['ResponseTime']/1000, df['Angle Error_deg_'], label='System')
# Add horizontal dashed red line at 2 degrees
plt.axhline(y=2, color='r', linestyle='--', label='Requirement (2 deg)')
plt.xlabel('Time (s)')
plt.ylabel('Angle Error (deg)')
plt.title('Angle Error from Direct Perpendicular (deg)')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
