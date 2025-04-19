import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV data
name = "100lbs_5-10deg_0.25Hz_PID"
df = pd.read_csv(r"Trimmed_Dynamic_Testing_Data/TRIMMED_tether_data_20250416_100lbs_5-10deg_0.25Hz_PID.csv")  # Replace with your actual CSV filename

# Define the folder path
folder = 'plots/' + name
# Create the folder if it doesn't exist
os.makedirs(folder, exist_ok=True)

# Convert UTC Time to datetime (optional for nicer plotting)
# df['Microseconds Since Start'] = pd.to_datetime(df['Microseconds Since Start'], errors='coerce')

# Create the time vector
time = df['ResponseTime'] / 1000

# Define your time range
t_start = time.iloc[0] # in seconds
t_end = time.iloc[-1] # in seconds
# t_start = 55 # in seconds
# t_end = 65   # in seconds

# Create a mask for the time range
mask = (time >= t_start) & (time <= t_end)

# Apply the mask to all vectors
time = time[mask]
teth1length = df['Tether1Length'][mask]
teth2length = df['Tether2Length'][mask]
teth3length = df['Tether3Length'][mask]
teth1ref_force = df['Tether1Force'][mask]
teth2ref_force = df['Tether2Force'][mask]
teth3ref_force = df['Tether3Force'][mask]
teth1loadcell_force = df['Loadcell1Force'][mask]
teth2loadcell_force = df['Loadcell2Force'][mask]
teth3loadcell_force = df['Loadcell3Force'][mask]
f_err = df['ForceError_lbf_'][mask]
ang_err = df['AngleError_deg_'][mask]
Xapex = (df['XApex'] * 12)[mask]
Yapex = (df['YApex'] * 12)[mask]
Zapex = (df['ZApex'] * 12)[mask]



size = (10, 4)
# Example 1: Plot Tether Lengths Over Time
plt.figure(figsize=size)
plt.plot(time, teth1length, label='Tether 1 Length')
plt.plot(time, teth2length, label='Tether 2 Length')
plt.plot(time, teth3length, label='Tether 3 Length')
plt.xlabel('Time (s)')
plt.ylabel('Tether Length (in)')
plt.title('Tether Lengths Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('plots/'+name+'/teth_lengths_'+name+'.png')


fig, axs = plt.subplots(3, 1, figsize=(10,8), sharex=True)

# Tether 1
axs[0].plot(time, teth1ref_force, label='Tether 1 Force')
axs[0].plot(time, teth1loadcell_force, label='Loadcell 1 Force')
axs[0].set_ylabel('Force (lbf)')
axs[0].set_title('Tether 1 Force Over Time')
axs[0].legend()
axs[0].grid(True)

# Tether 2
axs[1].plot(time, teth2ref_force, label='Tether 2 Force')
axs[1].plot(time, teth2loadcell_force, label='Loadcell 2 Force')
axs[1].set_ylabel('Force (lbf)')
axs[1].set_title('Tether 2 Force Over Time')
axs[1].legend()
axs[1].grid(True)

# Tether 3
axs[2].plot(time, teth3ref_force, label='Tether 3 Force')
axs[2].plot(time, teth3loadcell_force, label='Loadcell 3 Force')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Force (lbf)')
axs[2].set_title('Tether 3 Force Over Time')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.savefig('plots/'+name+'/tether_forces_subplot_' + name + '.png')

# Example 3: Apex Position (X, Y, Z)
plt.figure(figsize=size)
plt.plot(time, Xapex, label='X Apex')
plt.plot(time, Yapex, label='Y Apex')
plt.plot(time, Zapex, label='Z Apex')
plt.xlabel('Time (s)')
plt.ylabel('Position (in)')
plt.title('Assumed User CG Position Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('plots/'+name+'/apex_'+name+'.png')

# Example 4: Force Error (X, Y, Z) with requirement line
plt.figure(figsize=size)
plt.plot(time, f_err, label='System')
# Add horizontal dashed red line at 5 lbs
plt.axhline(y=5, color='r', linestyle='--', label='Requirement (5 lbf)')
plt.xlabel('Time (s)')
plt.ylabel('Force Magnitude Error (lbf)')
plt.title('Downward Force Magnitude Error')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('plots/'+name+'/force_err_'+name+'.png')

# Example 5: Angle Error (X, Y, Z) with requirement line
plt.figure(figsize=size)
plt.plot(time, ang_err, label='System')
# Add horizontal dashed red line at 2 degrees
plt.axhline(y=2, color='r', linestyle='--', label='Requirement (2 deg)')
plt.xlabel('Time (s)')
plt.ylabel('Angle Error (deg)')
plt.title('Angle Error from Direct Perpendicular (deg)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('plots/'+name+'/ang_err_'+name+'.png')

plt.show()
