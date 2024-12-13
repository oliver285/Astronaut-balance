import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import Parameters_2_Tether_Dynamic_Model


def equations(p, a, b, teth_anchor, offset):
    y, z = p
    return (
        # eft tether
        (y - (teth_anchor[0][0] + offset[0][0])) ** 2 + (z - (teth_anchor[0][1] + offset[0][1])) ** 2 - a ** 2,

        # right tether
        (y - (teth_anchor[1][0] + offset[1][0])) ** 2 + (z - (teth_anchor[1][1] + offset[1][1])) ** 2 - b ** 2,
    )


def calculate_apex(a, b, teth_anchor, offset):
    initial_guess = np.array([2, -2])  # Start with a point above the base
    apex = fsolve(equations, initial_guess, args=(a, b, teth_anchor, offset))
    return apex


def calculate_tether_vecs(COM, teth_anchor, offset):
    # determine the tether unit vector
    teth1_vec = np.array(teth_anchor[0][:]) - (np.array(COM) - np.array(offset[0][:]))
    teth2_vec = np.array(teth_anchor[1][:]) - (np.array(COM) - np.array(offset[1][:]))
    teth1_hat = teth1_vec / np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec / np.linalg.norm(teth2_vec)
    lengths = np.array([np.linalg.norm(teth1_vec), np.linalg.norm(teth2_vec)])

    return (teth1_hat, teth2_hat, lengths)


def calculate_tether_forces(apex, mass, teth_anchor, offset):
    # solve the system of eqns of the unit vectors to find the equations
    teth1_hat, teth2_hat, _ = calculate_tether_vecs(apex, teth_anchor, offset)

    M1 = np.array([[teth1_hat[0], teth2_hat[0]],
                   [teth1_hat[1], teth2_hat[1]]])
    M2 = np.array([[0], [mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f


def calculate_tether_error(COM, f, mass, teth_anchor, offset):
    teth1_hat, teth2_hat, _ = calculate_tether_vecs(COM, teth_anchor, offset)
    teth1_vec = f[0] * teth1_hat
    teth2_vec = f[1] * teth2_hat

    expected_f_vec = np.array([0, mass])
    f_vec = teth1_vec + teth2_vec

    # find the angle between expected and actual
    angle_err = np.arccos(np.dot(expected_f_vec, f_vec) / (np.linalg.norm(expected_f_vec) * np.linalg.norm(f_vec))) * (
                180 / np.pi)
    # determine the difference between their forces
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

    return f_err, angle_err, teth1_vec, teth2_vec


def simulate_tilting_motion(time_steps, dt, initial_height, tilt_axis):
    time = np.linspace(0, time_steps*dt, time_steps)
    
    # Initialize tilt angles
    x_tilt = np.zeros_like(time)
    y_tilt = np.zeros_like(time)
    z_trans = np.zeros_like(time)
    
    # Generate tilting angles (converting to radians) for specified axis
    # 0.75 is the sway requirement in hertz
    if tilt_axis.lower() == 'y':
        y_tilt = np.deg2rad(10) * np.sin(2 * np.pi * 0.75 * time)  # ±10 degrees in y
    elif tilt_axis.lower() == 'z':
        z_trans = 0.82021 * np.sin(2 * np.pi * 0.75 * time)
    else:
        raise ValueError("tilt_axis must be either 'y' or 'z'")
    
    positions = np.zeros((time_steps, 2))

    for i in range(time_steps):

        # Y position changes with side-to-side tilt (y_tilt)
        y = abs(initial_height) * np.sin(y_tilt[i])

        # Z position decreases as person tilts
        z = (initial_height+ z_trans[i]) * np.cos(y_tilt[i])

        positions[i] = [y, z]
    
    return positions, y_tilt

def plot_simulation_results(time_vec, positions, angles, f_errors, ang_errors, apex_error, torques, tilt_axis, tether1vec, tether2vec, err_teth_one_vec, err_teth_two_vec, max_hip_tilt):
    # Set figure size and style for all plots
    plt.style.use('default')
    figsize = (8, 4)

    p = Parameters_2_Tether_Dynamic_Model.Parameters()

    norm_time = (time_vec - time_vec.min()) / (time_vec.max() - time_vec.min())
    
    # 3D trajectory plot
    fig1 = plt.figure(figsize=figsize)
    ax1 = fig1.add_subplot(111)
    ax1.plot(positions[:, 0], positions[:, 1], color='k', linewidth=3)
    ax1.set_xlabel('Y (ft)')
    ax1.set_ylabel('Z (ft)')
    ax1.set_title('COM Trajectory')
    ax1.set_xlim([-2, 2])
    ax1.set_ylim([-5, 5])

    ax1.grid(True)
    plt.tight_layout()

    
    # Tilt angle plot
    fig2 = plt.figure(figsize=figsize)
    ax2 = fig2.add_subplot(111)
    ax2.plot(time_vec, np.rad2deg(angles), label='Y Tilt')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Tilt Angle (deg)')
    ax2.set_title(f'{tilt_axis.upper()}-Axis Tilt Angle Over Time')
    ax2.grid(True)
    ax2.legend()
    ax2.set_ylim([-15, 15])
    plt.tight_layout()

    
    # Position components plot
    fig3 = plt.figure(figsize=figsize)
    ax3 = fig3.add_subplot(111)
    ax3.plot(time_vec, positions[:, 0], label='Y')
    ax3.plot(time_vec, positions[:, 1], label='Z')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (ft)')
    ax3.set_title(f'Position Components Over Time, Tilt Axis: {tilt_axis}')
    ax3.grid(True)
    ax3.legend()
    ax3.set_ylim([-5, 5])
    plt.tight_layout()
    fig3.savefig(f'plots_report/2_tether_position_{tilt_axis}_mass_{p.mass}.png')
    
    # Force error plot
    fig4 = plt.figure(figsize=figsize)
    ax4 = fig4.add_subplot(111)
    ax4.plot(time_vec, f_errors)
    ax4.axhline(y=5, color='r', linestyle=':', label='Max Allowable Error (5 lbf)', linewidth = 3)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Force Error (lbf)')
    ax4.set_title(f'Tether Force Error, Max Hip Tilt = {max_hip_tilt} deg, Tilt Axis: {tilt_axis}')
    ax4.grid(True)
    ax4.legend()
    ax4.set_ylim([0, 8])
    plt.tight_layout()
    fig4.savefig(f'plots_report/2_tether_force_error_{tilt_axis}_mass_{p.mass}.png')
    
    # Angular error plot
    fig5 = plt.figure(figsize=figsize)
    ax5 = fig5.add_subplot(111)
    ax5.plot(time_vec, ang_errors)
    ax5.axhline(y=2, color='r', linestyle=':', label='Max Allowable Error (2 deg)', linewidth = 3)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angular Error (deg)')
    ax5.set_title(f'Tether Angular Error, Max Hip Tilt = {max_hip_tilt} deg, Tilt Axis: {tilt_axis}')
    ax5.grid(True)
    ax5.legend()
    ax5.set_ylim([0, 4])
    plt.tight_layout()
    fig5.savefig(f'plots_report/2_tether_angular_error_{tilt_axis}_mass_{p.mass}.png')

    # apex error plot
    fig15 = plt.figure(figsize=figsize)
    ax15 = fig15.add_subplot(111)
    ax15.plot(time_vec, apex_error)
    ax15.set_xlabel('Time (s)')
    ax15.set_ylabel('apex error (ft)')
    ax15.set_title('Apex Error')
    ax15.grid(True)
    ax15.set_ylim([0, 0.5])
    plt.tight_layout()
    

    # Tether force component plots 
    # Tether 1
    fig7 = plt.figure(figsize=figsize)
    ax7 = fig7.add_subplot(311)
    ax7.plot(time_vec, tether1vec[:, 0], label='Y')
    ax7.plot(time_vec, tether1vec[:, 1], label='Z')

    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Tether 1 Force Components (lbf)')
    ax7.grid(True)
    ax7.legend()
    ax7.set_ylim([-200, 200])

   
    
    # Tether 2
    ax8 = fig7.add_subplot(312)
    ax8.plot(time_vec, tether2vec[:, 0], label='Y')
    ax8.plot(time_vec, tether2vec[:, 1], label='Z')
   
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Tether 2 Force Components (lbf)')
    ax8.grid(True)
    ax8.set_ylim([-200, 200])
    

  

    fig10 = plt.figure(figsize=figsize)
    ax11 = fig10.add_subplot(311)
    ax12 = fig10.add_subplot(312)
    ax11.plot(time_vec, err_teth_one_vec * 12, color = 'r')
    ax12.plot(time_vec, err_teth_two_vec * 12, color = 'g')
    ax11.set_ylabel('Tether 1 Length Error (in)')
    ax12.set_ylabel('Tether 2 Length Error (in)')
    fig10.suptitle(f'Tether Length Errors (Actual - Assumed), Max Hip Tilt = {max_hip_tilt} deg, Tilt Axis: {tilt_axis}')
    for ax in [ax11, ax12]:
        ax.set_xlabel('Time (s)')
        ax.grid(True)
        ax.set_ylim([-3, 3])
    fig10.savefig(f'plots_report/2_tether_length_error_{tilt_axis}_mass_{p.mass}.png')
     

    plt.tight_layout()

def calculate_tether_lengths_rotated(harness_angle, COM, tether1_horz_coord, tether2_horz_coord, tilt_axis):
   
    # Convert angle to radians
    cos_theta = np.cos(harness_angle)
    sin_theta = np.sin(harness_angle)
    
    # Ground attachment points
    right_ground = np.array([2, 0])
    left_ground = np.array([-2, 0])
    
    
    # Rotation matrix for Y-axis rotation
    def rotate_point_around_com(point, COM):
        # Translate point relative to COM
        point_relative = point - COM
        if tilt_axis == 'y':
            # Rotate around x-axis
            y, z = point_relative
            new_y = y * cos_theta - z * sin_theta
            new_z = y * sin_theta + z * cos_theta
        elif tilt_axis == 'z':
            y,z = point_relative
            new_y = y
            new_z = z
        return np.array([new_y, new_z]) + COM
    # Rotate harness points around COM
    right_harness_rotated = rotate_point_around_com(tether1_horz_coord, COM)
    left_harness_rotated = rotate_point_around_com(tether2_horz_coord, COM)

    
    # Calculate new tether lengths
    right_length = np.linalg.norm(right_harness_rotated - right_ground)
    left_length = np.linalg.norm(left_harness_rotated - left_ground)
    
    return right_length, left_length, right_harness_rotated, left_harness_rotated

def main():
    p = Parameters_2_Tether_Dynamic_Model.Parameters()
    # Simulation parameters
    mass = p.mass  # person's weight (lb)
    initial_height = p.waist_height
    teth_anchor = p.teth_anchor
    offset = p.offset
    max_hip_tilt = 10
    tilt_axis = 'y'
    # Time parameters
    duration = 3.0  # seconds (5 complete cycles at 1Hz)
    dt = p.dt # time step (essentially our sensor suite update rate)
    time_steps = int(duration / dt) + 1
    time_vec = np.linspace(0, duration, time_steps)

    # force update rate parameters
    force_update_rate = p.force_update_rate  # time it takes to run tetrahedron calcs and update force command
    update_steps = int(duration / force_update_rate) + 1
    update_vec = np.linspace(0, duration, update_steps)
    # separate iterator for force update
    j = 0
    # linear assumption for time it takes servo to reach new torque
    reaction_t = p.reaction_t  # seconds
    # parameters defining old force and time for linear convergence
    f = np.array([[0.0], [0.0]])
    f_new = np.array([[0.0], [0.0]])
    f_old = np.array([[0.0], [0.0]])
    t1 = 0

    # Generate COM movement and tilt angles
    positions, tilt_angles = simulate_tilting_motion(time_steps, dt, initial_height, tilt_axis)

    # Initialize arrays to store results
    f_errors = np.zeros(time_steps)
    ang_errors = np.zeros(time_steps)
    torques = np.zeros(time_steps)
    apex_error = np.zeros(time_steps)
    # Arrays to store tether force vectors
    tether1_vec = np.zeros((time_steps, 2))
    tether2_vec = np.zeros((time_steps, 2))

    err_teth_one_vec = np.zeros(time_steps)
    err_teth_two_vec = np.zeros(time_steps)
    err_teth_three_vec = np.zeros(time_steps)
    teth1_len_rotated = np.zeros(time_steps)
    teth2_len_rotated = np.zeros(time_steps)
    teth3_len_rotated = np.zeros(time_steps)
    teth1_cord_rotated = np.zeros((time_steps,2))
    teth2_cord_rotated = np.zeros((time_steps,2))
    teth3_cord_rotated = np.zeros((time_steps, 2))
    time = np.linspace(0, time_steps*dt, time_steps)
    tilt_angle_rad = np.deg2rad(max_hip_tilt) * np.sin(2 * np.pi * 0.75 * time)
    # Run simulation
    for i in range(time_steps):
        # Update COM position based on tilt
        COM = positions[i, :]

        tether1_horz_coord = COM - offset[0]
        tether2_horz_coord = COM - offset[1]

        teth1_len_rotated[i], teth2_len_rotated[i], teth1_cord_rotated[i], teth2_cord_rotated[i] = calculate_tether_lengths_rotated(tilt_angle_rad[i], COM, tether1_horz_coord, tether2_horz_coord, tilt_axis)
        # Calculate tether properties
        _, _, teth_lengths = calculate_tether_vecs(COM, teth_anchor, offset)

        # tilte error included
        teth1_err = teth1_len_rotated[i] - teth_lengths[0]
        teth2_err = teth2_len_rotated[i] - teth_lengths[1]

        err_teth_one_vec[i] = teth1_err
        err_teth_two_vec[i] = teth2_err
    

        apex = calculate_apex(teth_lengths[0]+err_teth_one_vec[i], teth_lengths[1]+err_teth_two_vec[i], teth_anchor, offset)

        # update force at the start
        if i == 0:
            f = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_new = f
            f_old = f
            t1 = time_vec[i]
            j += 1
        # update force only at the prescribed update rate
        elif time_vec[i] >= update_vec[j]:
            # f = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_new = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_old = f
            t1 = time_vec[i]
            j += 1
        else:
            # if torque is not reached yet
            if (time_vec[i] < t1 + reaction_t) & (time_vec[i] > force_update_rate):
                f = ((f_new - f_old) / reaction_t) * (time_vec[i]) - ((f_new - f_old) / reaction_t) * t1 + f_old

            # if we reach torque before next update
            else:
                f = ((f_new - f_old) / reaction_t) * (t1 + reaction_t) - ((f_new - f_old) / reaction_t) * t1 + f_old

        # Calculate errors and torques
        f_err, ang_err, tether1_vec[i], tether2_vec[i] = calculate_tether_error(apex, f, mass, teth_anchor, offset)

        # Store results
        apex_error[i] = np.linalg.norm(COM-apex)
        f_errors[i] = f_err
        ang_errors[i] = ang_err
        # torques[i] = np.linalg.norm(calculate_applied_torque(COM, tilt_angles[i, 0], tilt_angles[i, 1], f, r))

    # Plot results
    plot_simulation_results(time_vec, positions, tilt_angles, f_errors, ang_errors, apex_error, torques, tilt_axis, tether1_vec, tether2_vec, err_teth_one_vec, err_teth_two_vec, max_hip_tilt)

    plt.show()
if __name__ == "__main__":
    main()
