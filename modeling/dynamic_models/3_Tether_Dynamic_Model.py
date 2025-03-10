# This model attempts to quantify the update rate needed by the controller to keep the angle and force errors under the
# threshold

# major assumptions:
#   uses the horizontal convergence method for doing the tetrahedron calcs which assumes the torso-tether attachment
#       plane is parallel to the floor and that the torso does not twist
#   assumes the tension change applied by the servos occurs in a linear time
#   assumes the torso is in the shape of a circle with radius r and the attachment location is at the same height as the
#       center of mass
#
# I am worried that this doesn't account for the fact that the actual control loop will run based off of changes in load
# cell readings that drive length changes rather than the other way around

import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import Parameters_3_Tether_Dynamic_Model
import time as timesec



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
    apex = fsolve(equations, np.array(initial_guess), args=(a, b, c, teth_anchor, offset))
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

    return (teth1_hat, teth2_hat, teth3_hat, lengths, teth1_vec, teth2_vec, teth3_vec)


def calculate_tether_forces(apex, mass, teth_anchor, offset):
    # solve the system of eqns of the unit vectors to find the equations
    teth1_hat, teth2_hat, teth3_hat, _, _, _, _ = calculate_tether_vecs(apex, teth_anchor, offset)

    M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                   [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                   [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
    M2 = np.array([[0],[0],[mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f


def calculate_tether_error(COM, f, mass, teth_anchor, offset):
    teth1_hat, teth2_hat, teth3_hat, _, _, _, _ = calculate_tether_vecs(COM, teth_anchor, offset)
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


def simulate_tilting_motion(time_steps, dt, initial_height, tilt_axis='x'):
    time = np.linspace(0, time_steps*dt, time_steps)
    
    # Initialize tilt angles
    x_tilt = np.zeros_like(time)
    y_tilt = np.zeros_like(time)
    z_trans = np.zeros_like(time)
    
    # Generate tilting angles (converting to radians) for specified axis
    # 0.75 is the sway requirement in hertz
    if tilt_axis.lower() == 'x':
        x_tilt = np.deg2rad(10) * np.sin(2 * np.pi * 0.75 * time)  # ±10 degrees in x
    elif tilt_axis.lower() == 'y':
        y_tilt = np.deg2rad(10) * np.sin(2 * np.pi * 0.75 * time)  # ±10 degrees in y
    elif tilt_axis.lower() == 'z':
        z_trans = (-0.82021 * np.cos(2 * np.pi * 0.75 * time) + 0.82021)/2
    else:
        raise ValueError("tilt_axis must be either 'x', 'y', or 'z'")
    
    positions = np.zeros((time_steps, 3))

    for i in range(time_steps):
        # Calculate new position based on tilt angles
        # X position changes with forward/backward tilt (x_tilt)
        x = abs(initial_height) * np.sin(x_tilt[i])

        # Y position changes with side-to-side tilt (y_tilt)
        y = abs(initial_height) * np.sin(y_tilt[i])

        # Z position decreases as person tilts
        z = (initial_height+ z_trans[i]) * np.cos(x_tilt[i]) * np.cos(y_tilt[i])

        positions[i] = [x, y, z]
    
    return positions, np.column_stack((x_tilt, y_tilt))

def plot_simulation_results(time_vec, positions, angles, f_errors, ang_errors, apex_error, torques, tilt_axis, tether1vec, tether2vec, tether3vec, err_teth_one_vec, err_teth_two_vec, err_teth_three_vec, max_hip_tilt, length_vec):
    # Set figure size and style for all plots
    plt.style.use('default')
    figsize = (8, 4)

    p = Parameters_3_Tether_Dynamic_Model.Parameters()

    norm_time = (time_vec - time_vec.min()) / (time_vec.max() - time_vec.min())
    
    # 3D trajectory plot
    fig1 = plt.figure(figsize=figsize)
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], color='k', linewidth=3)
    ax1.set_xlabel('X (ft)')
    ax1.set_ylabel('Y (ft)')
    ax1.set_zlabel('Z (ft)')
    ax1.set_title('COM Trajectory')
    ax1.set_xlim([-2, 2])
    ax1.set_ylim([-2, 2])
    ax1.set_zlim([-3, -1])
    ax1.invert_zaxis()
    ax1.grid(True)
    plt.tight_layout()

    
    # Tilt angle plot
    fig2 = plt.figure(figsize=figsize)
    ax2 = fig2.add_subplot(111)
    if tilt_axis.lower() == 'x':
        ax2.plot(time_vec, np.rad2deg(angles[:, 0]), label='X Tilt')
    else:
        ax2.plot(time_vec, np.rad2deg(angles[:, 1]), label='Y Tilt')
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
    ax3.plot(time_vec, positions[:, 0], label='X')
    ax3.plot(time_vec, positions[:, 1], label='Y')
    ax3.plot(time_vec, positions[:, 2], label='Z')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (ft)')
    ax3.set_title(f'Position Components Over Time, Tilt Axis: {tilt_axis}')
    ax3.grid(True)
    ax3.legend()
    ax3.set_ylim([-5, 5])
    plt.tight_layout()
    fig3.savefig(f'plots_report/3_tether_position_{tilt_axis}_mass_{p.mass}.png')

    # length components plot
    fig20 = plt.figure(figsize=figsize)
    ax20 = fig20.add_subplot(111)
    ax20.plot(time_vec, length_vec[:, 0], label='1')
    ax20.plot(time_vec, length_vec[:, 1], label='2')
    ax20.plot(time_vec, length_vec[:, 2], label='3')
    ax20.set_xlabel('Time (s)')
    ax20.set_ylabel('length (ft)')
    ax20.set_title(f'Tether lengths over time, Tilt Axis: {tilt_axis}')
    ax20.grid(True)
    ax20.legend()
    # ax15.set_ylim([-5, 5])
    plt.tight_layout()
    
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
    fig4.savefig(f'plots_report/3_tether_force_error_{tilt_axis}_mass_{p.mass}.png')
   
    
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
    fig5.savefig(f'plots_report/3_tether_angular_error_{tilt_axis}_mass_{p.mass}.png')
    

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
    ax7.plot(time_vec, tether1vec[:, 0], label='X')
    ax7.plot(time_vec, tether1vec[:, 1], label='Y')
    ax7.plot(time_vec, tether1vec[:, 2], label='Z')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Tether 1 Force Components (lbf)')
    ax7.grid(True)
    ax7.legend()
    ax7.set_ylim([-200, 200])

   
    
    # Tether 2
    ax8 = fig7.add_subplot(312)
    ax8.plot(time_vec, tether2vec[:, 0], label='X')
    ax8.plot(time_vec, tether2vec[:, 1], label='Y')
    ax8.plot(time_vec, tether2vec[:, 2], label='Z')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Tether 2 Force Components (lbf)')
    ax8.grid(True)
    ax8.set_ylim([-200, 200])
    
    
    # Tether 3
    ax9 = fig7.add_subplot(313)
    ax9.plot(time_vec, tether3vec[:, 0], label='X')
    ax9.plot(time_vec, tether3vec[:, 1], label='Y')
    ax9.plot(time_vec, tether3vec[:, 2], label='Z')
    ax9.set_xlabel('Time (s)')
    ax9.set_ylabel('Tether 3 Force Components (lbf)')
    ax9.grid(True)
    ax9.set_ylim([-200, 200])
    fig7.suptitle('Tether Force Vector Components Over Time')
    plt.tight_layout()
  

    fig10 = plt.figure(figsize=figsize)
    ax11 = fig10.add_subplot(311)
    ax12 = fig10.add_subplot(312)
    ax13 = fig10.add_subplot(313)
    ax11.plot(time_vec, err_teth_one_vec * 12, color = 'r')
    ax12.plot(time_vec, err_teth_two_vec * 12, color = 'g')
    ax13.plot(time_vec, err_teth_three_vec * 12, color = 'b')
    ax11.set_ylabel('Tether 1 Length Error (in)')
    ax12.set_ylabel('Tether 2 Length Error (in)')
    ax13.set_ylabel('Tether 3 Length Error (in)')
    fig10.suptitle(f'Tether Length Errors (Actual - Assumed), Max Hip Tilt = {max_hip_tilt} deg, Tilt Axis: {tilt_axis}')
    for ax in [ax11, ax12, ax13]:
        ax.set_xlabel('Time (s)')
        ax.grid(True)
        ax.set_ylim([-3, 3])
    fig10.savefig(f'plots_report/3_tether_length_error_{tilt_axis}_mass_{p.mass}.png')
    
     

    plt.tight_layout()

def calculate_tether_lengths_rotated(harness_angle, COM, tether1_horz_coord, tether2_horz_coord, tether3_horz_coord, tilt_axis):
   
    # Convert angle to radians
    cos_theta = np.cos(harness_angle)
    sin_theta = np.sin(harness_angle)
    
    # Ground attachment points
    front_ground = np.array([-2, 0, 0])
    back_left_ground = np.array([2, -2, 0])
    back_right_ground = np.array([2, 2, 0])
    
    # Rotation matrix for Y-axis rotation
    def rotate_point_around_com(point, COM):
        # Translate point relative to COM
        point_relative = point - COM
        if tilt_axis == 'x':
            # Rotate around y-axis
            x, y, z = point_relative
            new_x = x * cos_theta - z * sin_theta
            new_y = y
            new_z = x * sin_theta + z * cos_theta
        elif tilt_axis == 'y':
            # Rotate around x-axis
            x, y, z = point_relative
            new_x = x
            new_y = y * cos_theta - z * sin_theta
            new_z = y * sin_theta + z * cos_theta
        elif tilt_axis == 'z':
            x,y,z = point_relative
            new_x = x
            new_y = y
            new_z = z
        
        # Translate back
        return np.array([new_x, new_y, new_z]) + COM
    
    # Rotate harness points around COM
    front_harness_rotated = rotate_point_around_com(tether1_horz_coord, COM)
    back_left_harness_rotated = rotate_point_around_com(tether2_horz_coord, COM)
    back_right_harness_rotated = rotate_point_around_com(tether3_horz_coord, COM)
    
    # Calculate new tether lengths
    front_length = np.linalg.norm(front_harness_rotated - front_ground)
    back_left_length = np.linalg.norm(back_left_harness_rotated - back_left_ground)
    back_right_length = np.linalg.norm(back_right_harness_rotated - back_right_ground)
    
    return front_length, back_left_length, back_right_length, front_harness_rotated, back_left_harness_rotated, back_right_harness_rotated


def main():
    p = Parameters_3_Tether_Dynamic_Model.Parameters()
    # Simulation parameters
    mass = p.mass  # person's weight (lb)
    max_hip_tilt = 10
    initial_height = p.waist_height
    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other
    teth_anchor = p.teth_anchor
    # attachment offset vector 3x3 each row is the vector for each tether
    # assuming circular radius (pointing from tether attachment loc to COM
    offset = p.offset

    # range of tether error lengths
    teth_length_error_range = 0

    # Time parameters
    duration = 3.0  # seconds (5 complete cycles at 1Hz)
    dt = p.dt  # time step (essentially our sensor suite update rate)
    
    time_steps = int(duration/dt)+1
    time_vec = np.linspace(0, duration, time_steps)
 
    # force update rate parameters
    force_update_rate = p.force_update_rate  # time it takes to run tetrahedron calcs and update force command
    update_steps = int(duration / force_update_rate)+1
    update_vec = np.linspace(0, duration, update_steps)
    # separate iterator for force update
    j = 0
    # linear assumption for time it takes servo to reach new torque
    reaction_t = p.reaction_t  # seconds
    # parameters defining old force and time for linear convergence
    f = np.array([[0.0], [0.0], [0.0]])
    f_new = np.array([[0.0], [0.0], [0.0]])
    f_old = np.array([[0.0], [0.0], [0.0]])
    t1 = 0
    
    # Set tilt axis ('x' or 'y')
    tilt_axis = 'z'
    
    # Generate COM movement and tilt angles
    positions, tilt_angles = simulate_tilting_motion(time_steps, dt, initial_height, tilt_axis)
    apex_vec = np.zeros((time_steps, 3))
    
    # Initialize arrays to store results
    f_errors = np.zeros(time_steps)
    ang_errors = np.zeros(time_steps)
    apex_error = np.zeros(time_steps)
    torques = np.zeros(time_steps)
    
    # Arrays to store tether force vectors
    tether1_vec = np.zeros((time_steps, 3))
    tether2_vec = np.zeros((time_steps, 3))
    tether3_vec = np.zeros((time_steps, 3))
    length_vec = np.zeros((time_steps, 3))
    err_teth_one_vec = np.zeros(time_steps)
    err_teth_two_vec = np.zeros(time_steps)
    err_teth_three_vec = np.zeros(time_steps)
    teth1_length_horizontal = np.zeros(time_steps)
    teth2_length_horizontal = np.zeros(time_steps)
    teth3_length_horizontal = np.zeros(time_steps)
   
    teth1_len_rotated = np.zeros(time_steps)
    teth2_len_rotated = np.zeros(time_steps)
    teth3_len_rotated = np.zeros(time_steps)
    teth1_cord_rotated = np.zeros((time_steps,3))
    teth2_cord_rotated = np.zeros((time_steps,3))
    teth3_cord_rotated = np.zeros((time_steps, 3))
    tether1_horz_coord = np.zeros((time_steps, 3))
    teth1_rotated = np.zeros((time_steps,3))
    teth2_rotated = np.zeros((time_steps,3))
    time = np.linspace(0, time_steps*dt, time_steps)
    tilt_angle_rad = np.deg2rad(max_hip_tilt) * np.sin(2 * np.pi * 0.75 * time)
    
    # Run simulation
    for i in range(time_steps):
        # Update COM position based on tilt
        COM = positions[i, :]
        
        tether1_horz_coord[i] = COM - offset[0]
        tether2_horz_coord = COM - offset[1]
        tether3_horz_coord = COM - offset[2]
        teth1_len_rotated[i], teth2_len_rotated[i], teth3_len_rotated[i], teth1_cord_rotated[i], teth2_cord_rotated[i], teth3_cord_rotated[i] = calculate_tether_lengths_rotated(tilt_angle_rad[i], COM, tether1_horz_coord[i], tether2_horz_coord, tether3_horz_coord, tilt_axis)
        # Calculate tether properties
        _, _, _, teth_lengths, teth1_rotated[i], teth2_rotated[i], _= calculate_tether_vecs(COM, teth_anchor, offset)
        length_vec[i] = teth_lengths

        # tether error due to tilt
        teth1_err = teth1_len_rotated[i] - teth_lengths[0]
        teth2_err = teth2_len_rotated[i] - teth_lengths[1]
        teth3_err = teth3_len_rotated[i] - teth_lengths[2]
        
        err_teth_one_vec[i] = teth1_err
        err_teth_two_vec[i] = teth2_err
        err_teth_three_vec[i] = teth3_err
          
        apex = calculate_apex(teth_lengths[0]+err_teth_one_vec[i], teth_lengths[1]+err_teth_two_vec[i], teth_lengths[2]+err_teth_three_vec[i], teth_anchor, offset)
       

        apex_vec[i, :] = apex

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
            if (time_vec[i] < t1+reaction_t) & (time_vec[i] > force_update_rate):
                f = ((f_new - f_old) / reaction_t) * (time_vec[i]) - ((f_new - f_old) / reaction_t) * t1 + f_old

            # if we reach torque before next update
            else:
                f = ((f_new - f_old) / reaction_t) * (t1+reaction_t) - ((f_new - f_old) / reaction_t) * t1 + f_old

        # Calculate errors and torques
        f_err, ang_err, tether1_vec[i], tether2_vec[i], tether3_vec[i] = calculate_tether_error(apex, f, mass, teth_anchor, offset)

        # Store results
        apex_error[i] = np.linalg.norm(COM-apex)
        f_errors[i] = f_err
        ang_errors[i] = ang_err
        teth1_length_horizontal[i] = teth_lengths[0]
        teth2_length_horizontal[i] = teth_lengths[1]
        teth3_length_horizontal[i] = teth_lengths[2]
        # torques[i] = np.linalg.norm(calculate_applied_torque(COM, tilt_angles[i, 0], tilt_angles[i, 1], f, r))

    # Plot results
    plot_simulation_results(time_vec, positions, tilt_angles, f_errors, ang_errors, apex_error, torques, tilt_axis,
                            tether1_vec, tether2_vec, tether3_vec, err_teth_one_vec, err_teth_two_vec, err_teth_three_vec, max_hip_tilt, length_vec)
    
    plt.show()
   
if __name__ == "__main__":
    main()
