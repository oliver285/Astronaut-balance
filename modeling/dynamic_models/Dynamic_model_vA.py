# This model attempts to quantify the update rate needed by the controller to keep the angle and force errors under the
# threshold

# major assumptions:
#   uses the horizontal convergence method for doing the tetrahedron calcs which assumes the torso-tether attachment
#       plane is parallel to the floor and that the torso does not twist
#   assumes an immediate change in tension is possible from the servo motors, this adjustment time will be factored in
#       later
#   assumes the torso is in the shape of a circle with radius r and the attachment location is at the same height as the
#       center of mass
#
# I am worried that this doesn't account for the fact that the actual control loop will run based off of changes in load
# cell readings that drive length changes rather than the other way around

import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def equations(p, a, b, c, teth_anchor, offset):
    x, y, z = p
    return (
        # front tether
        (x - (teth_anchor[0][0] + offset[0][0])) ** 2 + (y - (teth_anchor[0][1] + offset[0][1])) ** 2 + (z - (teth_anchor[0][2] + offset[0][2])) ** 2 - a ** 2,

        # left tether
        (x - (teth_anchor[1][0] + offset[1][0])) ** 2 + (y - (teth_anchor[1][1] + offset[1][1])) ** 2 + (z - (teth_anchor[1][2] + offset[1][2])) ** 2 - b ** 2,

        # right tether
        (x - (teth_anchor[2][0] + offset[2][0])) ** 2 + (y - (teth_anchor[2][1] + offset[2][1])) ** 2 + (z - (teth_anchor[2][2] + offset[2][2])) ** 2 - c ** 2,
    )


def calculate_apex(a, b, c, teth_anchor, offset):
    initial_guess = [2, 2, -4]  # Start with a point above the base
    apex = fsolve(equations, initial_guess, args=(a, b, c, teth_anchor, offset))
    return apex


def calculate_tether_vecs(COM, teth_anchor, offset):
    # determine the tether unit vector
    teth1_vec = np.array(teth_anchor[0][:]) - (np.array(COM) - np.array(offset[0][:]))
    teth2_vec = np.array(teth_anchor[1][:]) - (np.array(COM) - np.array(offset[1][:]))
    teth3_vec = np.array(teth_anchor[2][:]) - (np.array(COM) - np.array(offset[2][:]))
    teth1_hat = teth1_vec/np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec/np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec/np.linalg.norm(teth3_vec)
    lengths = np.array([np.linalg.norm(teth1_vec),np.linalg.norm(teth2_vec),np.linalg.norm(teth3_vec)])

    return (teth1_hat, teth2_hat, teth3_hat, lengths)


def calculate_tether_forces(apex, mass, teth_anchor, offset):
    # solve the system of eqns of the unit vectors to find the equations
    teth1_hat, teth2_hat, teth3_hat, _ = calculate_tether_vecs(apex, teth_anchor, offset)

    M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                   [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                   [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
    M2 = np.array([[0],[0],[mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f


def calculate_tether_error(COM, f, mass, teth_anchor, offset):
    teth1_hat, teth2_hat, teth3_hat, _ = calculate_tether_vecs(COM, teth_anchor, offset)
    teth1_vec = f[0]*teth1_hat
    teth2_vec = f[1]*teth2_hat
    teth3_vec = f[2]*teth3_hat

    expected_f_vec = np.array([0, 0, mass])
    f_vec = teth1_vec + teth2_vec + teth3_vec

    # find the angle between expected and actual
    angle_err = np.arccos(np.dot(expected_f_vec, f_vec)/(np.linalg.norm(expected_f_vec)*np.linalg.norm(f_vec)))*(180/np.pi)
    # determine the difference between their forces
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

    return f_err, angle_err, teth1_vec,teth2_vec, teth3_vec


def simulate_tilting_motion(time_steps, dt, tilt_axis='x'):
    time = np.linspace(0, time_steps*dt, time_steps)
    
    # Initial position and height
    initial_height = -3.0  # feet
    
    # Initialize tilt angles
    x_tilt = np.zeros_like(time)
    y_tilt = np.zeros_like(time)
    
    # Generate tilting angles (converting to radians) for specified axis
    # 0.75 is the sway requirement in hertz
    if tilt_axis.lower() == 'x':
        x_tilt = np.deg2rad(10) * np.sin(2 * np.pi * 0.75 * time)  # ±10 degrees in x
    elif tilt_axis.lower() == 'y':
        y_tilt = np.deg2rad(10) * np.sin(2 * np.pi * 0.75 * time)  # ±10 degrees in y
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
    ax2.set_ylabel('Tilt Angle (deg)')
    ax2.set_title(f'{tilt_axis.upper()}-Axis Tilt Angle Over Time')
    ax2.legend()
    
    # Position components plot
    ax3 = fig.add_subplot(323)
    ax3.plot(time_vec, positions[:, 0], label='X')
    ax3.plot(time_vec, positions[:, 1], label='Y')
    ax3.plot(time_vec, positions[:, 2], label='Z')
    ax3.set_ylabel('Position (ft)')
    ax3.set_title('Position Components Over Time')
    ax3.legend()
    
    # Force error plot
    ax4 = fig.add_subplot(324)
    ax4.plot(time_vec, f_errors)
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
    teth_percent_error = 0.05  # percent error in tether length

    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other
    teth_anchor = [[2.0,                      0.0,                     0.0],
                  [2.0*np.cos(240*np.pi/180), 2.0*np.sin(240*np.pi/180), 0.0],
                  [2.0*np.cos(120*np.pi/180), 2.0*np.sin(120*np.pi/180), 0.0]]
    # attachment offset vector 3x3 each row is the vector for each tether
    # assuming circular radius (pointing from tether attachment loc to COM
    offset = [[-r,                      0.0,                     0.0],
              [-r*np.cos(240*np.pi/180), -r*np.sin(240*np.pi/180), 0.0],
              [-r*np.cos(120*np.pi/180), -r*np.sin(120*np.pi/180), 0.0]]

    # Time parameters
    duration = 5.0  # seconds (5 complete cycles at 1Hz)
    dt = 0.001  # time step (essentially our sensor suite update rate)
    time_steps = int(duration/dt)+1
    time_vec = np.linspace(0, duration, time_steps)

    # force update rate parameters
    force_update_rate = 0.002  # time it takes to run tetrahedron calcs and update force command
    update_steps = int(duration / force_update_rate)+1
    update_vec = np.linspace(0, duration, update_steps)
    # separate iterator for force update
    j = 0
    # linear assumption for time it takes servo to reach new torque
    reaction_t = 0.01  # seconds
    # parameters defining old force and time for linear convergence
    f = np.array([[0.0], [0.0], [0.0]])
    f_new = np.array([[0.0], [0.0], [0.0]])
    f_old = np.array([[0.0], [0.0], [0.0]])
    t1 = 0
    
    # Set tilt axis ('x' or 'y')
    tilt_axis = 'x'
    
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
        _, _, _, teth_lengths = calculate_tether_vecs(COM, teth_anchor, offset)

        teth_lengths = teth_lengths + teth_percent_error * teth_lengths
        apex = calculate_apex(teth_lengths[0], teth_lengths[1], teth_lengths[2], teth_anchor, offset)

        # update force at the start
        if i == 0:
            f = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_new = f
            f_old = f
            t1 = time_vec[i]
            j += 1
        # update force only at the prescribed update rate
        elif abs(time_vec[i] - update_vec[j]) < dt/2:
            # f = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_new = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_old = f
            t1 = time_vec[i]
            j += 1
        else:
            # if torque is not reached yet
            if (time_vec[i] < t1+reaction_t) & (time_vec[i] > force_update_rate):
                f = ((f_new - f_old) / reaction_t) * (time_vec[i]) - ((f_new - f_old) / reaction_t) * t1 + f_old

            # if we reach torque before next update
            else:
                f = ((f_new - f_old) / reaction_t) * (t1+reaction_t) - ((f_new - f_old) / reaction_t) * t1 + f_old

        # Calculate errors and torques
        f_err, ang_err, tether1_vec[i], tether2_vec[i], tether3_vec[i] = calculate_tether_error(COM, f, mass, teth_anchor, offset)
        
        # Store results
        f_errors[i] = f_err
        ang_errors[i] = ang_err
        # torques[i] = np.linalg.norm(calculate_applied_torque(COM, tilt_angles[i, 0], tilt_angles[i, 1], f, r))
    
    # Plot results
    plot_simulation_results(time_vec, positions, tilt_angles, f_errors, ang_errors, torques, tilt_axis, tether1_vec, tether2_vec, tether3_vec)
    print(np.sqrt((tether1_vec[0][0])**2+(tether1_vec[0][1])**2+(tether1_vec[0][2])**2))
    # print(np.sqrt((tether2_vec[0][0])**2+(tether2_vec[0][1])**2+(tether2_vec[0][2])**2))
    # print(np.sqrt((tether3_vec[0][0])**2+(tether3_vec[0][1])**2+(tether3_vec[0][2])**2))


if __name__ == "__main__":
    main()
