import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def equations(p, a, b, c, r):
    x, y, z = p
    return (
        x**2 + (y-(2-r))**2 + z**2 - a**2,
        (x-(2-r)*np.cos(210*np.pi/180))**2 + (y-(2-r)*np.sin(210*np.pi/180))**2 + z**2 - b**2,
        (x-(2-r)*np.cos(330*np.pi/180))**2 + (y-(2-r)*np.sin(330*np.pi/180))**2 + z**2 - c**2
    )

def calculate_apex(a, b, c, r):
    initial_guess = [2, 2, 4]  # Start with a point above the base
    apex = fsolve(equations, initial_guess, args=(a, b, c, r))
    return apex

def calculate_tether_forces(apex, mass, r):
    teth1_vec = np.array(apex) - np.array([0, (2-r), 0])
    teth2_vec = np.array(apex) - np.array([(2-r)*np.cos(210*np.pi/180), (2-r)*np.sin(210*np.pi/180), 0])
    teth3_vec = np.array(apex) - np.array([(2-r)*np.cos(330*np.pi/180), (2-r)*np.sin(330*np.pi/180), 0])
    teth1_hat = teth1_vec/np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec/np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec/np.linalg.norm(teth3_vec)
    M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                   [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                   [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
    M2 = np.array([[0],[0],[mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f

def calculate_tether_length(COM, r):
    teth1_attach_loc = np.array(COM) + np.array([0.0, r, 0.0])
    teth2_attach_loc = np.array(COM) + np.array([r*np.cos(210*np.pi/180), r*np.sin(210*np.pi/180), 0])
    teth3_attach_loc = np.array(COM) + np.array([r*np.cos(330*np.pi/180), r*np.sin(330*np.pi/180), 0])
    teth_lengths = np.array([0.0, 0.0, 0.0])
    teth_lengths[0] = np.linalg.norm(teth1_attach_loc - np.array([0.0, 2.0, 0.0]))
    teth_lengths[1] = np.linalg.norm(teth2_attach_loc - np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0]))
    teth_lengths[2] = np.linalg.norm(teth3_attach_loc - np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0]))
    return teth_lengths

def calculate_tether_error(COM, f, mass, r):
    teth1_attach_loc = np.array(COM) + np.array([0.0, r, 0.0])
    teth2_attach_loc = np.array(COM) + np.array([r*np.cos(210*np.pi/180), r*np.sin(210*np.pi/180), 0])
    teth3_attach_loc = np.array(COM) + np.array([r*np.cos(330*np.pi/180), r*np.sin(330*np.pi/180), 0])
    teth1_init_vec = np.array([0.0, 2.0, 0.0]) - teth1_attach_loc
    teth2_init_vec = np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0]) - teth2_attach_loc
    teth3_init_vec = np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0]) - teth3_attach_loc
    teth1_vec = f[0]*(teth1_init_vec/np.linalg.norm(teth1_init_vec))
    teth2_vec = f[1]*(teth2_init_vec/np.linalg.norm(teth2_init_vec))
    teth3_vec = f[2]*(teth3_init_vec/np.linalg.norm(teth3_init_vec))

    expected_f_vec = np.array([0, 0, -mass])
    f_vec = teth1_vec + teth2_vec + teth3_vec

    angle_err = np.arccos(np.dot(expected_f_vec, f_vec)/(np.linalg.norm(expected_f_vec)*np.linalg.norm(f_vec)))*(180/np.pi)
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

    return f_err, angle_err, teth1_vec,teth2_vec, teth3_vec

def simulate_tilting_motion(time_steps, dt, tilt_axis='x'):
    time = np.linspace(0, time_steps*dt, time_steps)
    
    # Initial position and height
    initial_height = 3.0  # feet
    
    # Initialize tilt angles
    x_tilt = np.zeros_like(time)
    y_tilt = np.zeros_like(time)
    
    # Generate tilting angles (converting to radians) for specified axis
    if tilt_axis.lower() == 'x':
        x_tilt = np.deg2rad(10) * np.sin(2 * np.pi * time)  # ±10 degrees in x
    elif tilt_axis.lower() == 'y':
        y_tilt = np.deg2rad(10) * np.sin(2 * np.pi * time)  # ±10 degrees in y
    else:
        raise ValueError("tilt_axis must be either 'x' or 'y'")
    
    positions = np.zeros((time_steps, 3))
    
    for i in range(time_steps):
        # Calculate new position based on tilt angles
        # X position changes with forward/backward tilt (x_tilt)
        x = initial_height * np.sin(x_tilt[i])
        
        # Y position changes with side-to-side tilt (y_tilt)
        y = initial_height * np.sin(y_tilt[i])
        
        # Z position decreases as person tilts
        z = initial_height * np.cos(x_tilt[i]) * np.cos(y_tilt[i])
        
        positions[i] = [x, y, z]
    
    return positions, np.column_stack((x_tilt, y_tilt))

def plot_simulation_results(time_vec, positions, angles, f_errors, ang_errors, torques, tilt_axis, tether1vec, tether2vec, tether3vec):
    fig = plt.figure(figsize=(15, 12))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(321, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2])
    ax1.set_xlabel('X (ft)')
    ax1.set_ylabel('Y (ft)')
    ax1.set_zlabel('Z (ft)')
    ax1.set_title('COM Trajectory')
    
    # Tilt angle plot
    ax2 = fig.add_subplot(322)
    if tilt_axis.lower() == 'x':
        ax2.plot(time_vec, np.rad2deg(angles[:, 0]), label='X Tilt')
    else:
        ax2.plot(time_vec, np.rad2deg(angles[:, 1]), label='Y Tilt')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Tilt Angle (deg)')
    ax2.set_title(f'{tilt_axis.upper()}-Axis Tilt Angle Over Time')
    ax2.legend()
    
    # Position components plot
    ax3 = fig.add_subplot(323)
    ax3.plot(time_vec, positions[:, 0], label='X')
    ax3.plot(time_vec, positions[:, 1], label='Y')
    ax3.plot(time_vec, positions[:, 2], label='Z')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (ft)')
    ax3.set_title('Position Components Over Time')
    ax3.legend()
    
    # Force error plot
    ax4 = fig.add_subplot(324)
    ax4.plot(time_vec, f_errors)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Force Error (lbf)')
    ax4.set_title('Tether Force Error')
    
    # Angular error plot
    ax5 = fig.add_subplot(325)
    ax5.plot(time_vec, ang_errors)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angular Error (deg)')
    ax5.set_title('Tether Angular Error')
    
    # Torque magnitude plot
    ax6 = fig.add_subplot(326)
    ax6.plot(time_vec, torques)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Torque (lb*ft)')
    ax6.set_title('Total Applied Torque')
    
     # Tether force component plots
    fig2, (ax7, ax8, ax9) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    ax7.plot(time_vec, tether1vec[:, 0], label='Tether 1 X')
    ax7.plot(time_vec, tether1vec[:, 1], label='Tether 1 Y')
    ax7.plot(time_vec, tether1vec[:, 2], label='Tether 1 Z')
    ax7.set_ylabel('Tether 1 Force (lbf)')
    ax7.legend()
    ax8.plot(time_vec, tether2vec[:, 0], label='Tether 2 X')
    ax8.plot(time_vec, tether2vec[:, 1], label='Tether 2 Y')
    ax8.plot(time_vec, tether2vec[:, 2], label='Tether 2 Z')
    ax8.set_ylabel('Tether 2 Force (lbf)')
    ax8.legend()
    ax9.plot(time_vec, tether3vec[:, 0], label='Tether 3 X')
    ax9.plot(time_vec, tether3vec[:, 1], label='Tether 3 Y')
    ax9.plot(time_vec, tether3vec[:, 2], label='Tether 3 Z')
    ax9.set_xlabel('Time (s)')
    ax9.set_ylabel('Tether 3 Force (lbf)')
    ax9.legend()
    fig2.suptitle('Tether Force Components Over Time')
    


    plt.tight_layout()
    plt.show()



def main():
    # Simulation parameters
    mass = 200.0  # person's weight (lb)
    r = 3 / (2 * np.pi)  # waist radius (ft)
    
    # Time parameters
    duration = 5.0  # seconds (5 complete cycles at 1Hz)
    dt = 0.01  # time step
    time_steps = int(duration/dt)
    time_vec = np.linspace(0, duration, time_steps)
    
    # Set tilt axis ('x' or 'y')
    tilt_axis = 'y'  
    
    # Generate COM movement and tilt angles
    positions, tilt_angles = simulate_tilting_motion(time_steps, dt, tilt_axis)
    
    # Initialize arrays to store results
    f_errors = np.zeros(time_steps)
    ang_errors = np.zeros(time_steps)
    torques = np.zeros(time_steps)
    
    # Arrays to store tether force vectors
    tether1_vec = np.zeros((time_steps, 3))
    tether2_vec = np.zeros((time_steps, 3))
    tether3_vec = np.zeros((time_steps, 3))
    
    # Run simulation
    for i in range(time_steps):
        # Update COM position based on tilt
        COM = positions[i, :]
        
        # Calculate tether properties
        teth_lengths = calculate_tether_length(COM, r)
        apex = calculate_apex(teth_lengths[0], teth_lengths[1], teth_lengths[2], r)
        f = calculate_tether_forces(apex, mass, r)
        
        # Calculate errors and torques
        f_err, ang_err, tether1_vec[i], tether2_vec[i], tether3_vec[i] = calculate_tether_error(COM, f, mass, r)
        
        # Limit angular error to less than 5 degrees
        if ang_err > 5:
            # Adjust tether forces to reduce angular error
            f = calculate_tether_forces(apex, mass, r)
            f_err, ang_err, tether1_vec[i], tether2_vec[i], tether3_vec[i] = calculate_tether_error(COM, f, mass, r)
        
        # Store results
        f_errors[i] = f_err
        ang_errors[i] = ang_err
        #torques[i] = np.linalg.norm(calculate_applied_torque(COM, tilt_angles[i, 0], tilt_angles[i, 1], f, r))
    
    # Plot results
    plot_simulation_results(time_vec, positions, tilt_angles, f_errors, ang_errors, torques, tilt_axis, tether1_vec, tether2_vec, tether3_vec)
    print(np.sqrt((tether1_vec[0][0])**2+(tether1_vec[0][1])**2+(tether1_vec[0][2])**2))
    print(np.sqrt((tether2_vec[0][0])**2+(tether2_vec[0][1])**2+(tether2_vec[0][2])**2))
    print(np.sqrt((tether3_vec[0][0])**2+(tether3_vec[0][1])**2+(tether3_vec[0][2])**2))


if __name__ == "__main__":
    main()
