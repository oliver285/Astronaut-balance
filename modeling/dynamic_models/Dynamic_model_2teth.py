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
import Parameters


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


def simulate_tilting_motion(time_steps, dt):
    time = np.linspace(0, time_steps * dt, time_steps)

    # Initial position and height
    initial_height = -3.0  # feet

    # Initialize tilt angles
    y_tilt = np.deg2rad(10) * np.sin(2 * np.pi * 0.75 * time)  # ±10 degrees in y

    positions = np.zeros((time_steps, 2))

    for i in range(time_steps):
        # Calculate new position based on tilt angles

        # Y position changes with side-to-side tilt (y_tilt)
        y = initial_height * np.sin(y_tilt[i])

        # Z position decreases as person tilts
        z = initial_height * np.cos(y_tilt[i])

        positions[i] = [y, z]

    return positions, y_tilt


def plot_simulation_results(time_vec, positions, angles, f_errors, ang_errors, torques, tether1vec, tether2vec):
    fig = plt.figure(figsize=(15, 12))

    # 3D trajectory plot
    ax1 = fig.add_subplot(321)
    ax1.plot(positions[:, 0], -positions[:, 1])
    ax1.set_xlabel('Y (ft)')
    ax1.set_ylabel('Z (ft)')
    ax1.set_title('COM Trajectory')

    # Tilt angle plot
    ax2 = fig.add_subplot(322)
    ax2.plot(time_vec, np.rad2deg(angles), label='Y Tilt')
    ax2.set_ylabel('Tilt Angle (deg)')
    ax2.set_title(f'Y-Axis Tilt Angle Over Time')
    ax2.legend()

    # Position components plot
    ax3 = fig.add_subplot(323)
    ax3.plot(time_vec, positions[:, 0], label='Y')
    ax3.plot(time_vec, positions[:, 1], label='Z')
    ax3.set_ylabel('Position (ft)')
    ax3.set_title('Position Components Over Time')
    ax3.legend()

    # Force error plot
    fig4 = plt.figure(figsize=(10,8))
    ax4 = fig4.add_subplot(111)
    ax4.plot(time_vec, f_errors)
    ax4.axhline(y=5, color='r', linestyle=':', label='Max Allowable Error (5 lbf)', linewidth = 3)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Force Error (lbf)')
    ax4.set_title('Tether Force Error')
    ax4.grid(True)
    ax4.legend()
    ax4.set_ylim([0, 8])
    plt.tight_layout()
    plt.savefig('plots/force_error_2teth.png')

    # Angular error plot
    fig5 = plt.figure(figsize=(10,8))
    ax5 = fig5.add_subplot(111)
    ax5.plot(time_vec, ang_errors)
    ax5.axhline(y=2, color='r', linestyle=':', label='Max Allowable Error (2 deg)', linewidth = 3)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angular Error (deg)')
    ax5.set_title('Tether Angular Error')
    ax5.grid(True)
    ax5.legend()
    ax5.set_ylim([0, 4])
    plt.tight_layout()
    plt.savefig('plots/angular_error_2teth.png')

    # Torque magnitude plot
    ax6 = fig.add_subplot(326)
    ax6.plot(time_vec, torques)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Torque (lb*ft)')
    ax6.set_title('Total Applied Torque')

    # Tether force component plots
    fig2, (ax7, ax8) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    ax7.plot(time_vec, tether1vec[:, 0], label='Tether 1 Y')
    ax7.plot(time_vec, tether1vec[:, 1], label='Tether 1 Z')
    ax7.set_ylabel('Tether 1 Force (lbf)')
    ax7.legend()
    ax8.plot(time_vec, tether2vec[:, 0], label='Tether 2 Y')
    ax8.plot(time_vec, tether2vec[:, 1], label='Tether 2 Z')
    ax8.set_ylabel('Tether 2 Force (lbf)')
    ax8.legend()
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
    teth_anchor = [[2.0, 0.0],
                   [-2.0, 0.0]]
    # attachment offset vector 3x3 each row is the vector for each tether
    # assuming circular radius (pointing from tether attachment loc to COM
    offset = [[-r, 0.0],
              [r, 0.0]]

    # Time parameters
    duration = 5.0  # seconds (5 complete cycles at 1Hz)
    dt = 0.001  # time step (essentially our sensor suite update rate)
    time_steps = int(duration / dt) + 1
    time_vec = np.linspace(0, duration, time_steps)

    # force update rate parameters
    force_update_rate = 0.002  # time it takes to run tetrahedron calcs and update force command
    update_steps = int(duration / force_update_rate) + 1
    update_vec = np.linspace(0, duration, update_steps)
    # separate iterator for force update
    j = 0
    # linear assumption for time it takes servo to reach new torque
    reaction_t = 0.01  # seconds
    # parameters defining old force and time for linear convergence
    f = np.array([[0.0], [0.0]])
    f_new = np.array([[0.0], [0.0]])
    f_old = np.array([[0.0], [0.0]])
    t1 = 0

    # Generate COM movement and tilt angles
    positions, tilt_angles = simulate_tilting_motion(time_steps, dt)

    # Initialize arrays to store results
    f_errors = np.zeros(time_steps)
    ang_errors = np.zeros(time_steps)
    torques = np.zeros(time_steps)

    # Arrays to store tether force vectors
    tether1_vec = np.zeros((time_steps, 2))
    tether2_vec = np.zeros((time_steps, 2))

    # Run simulation
    for i in range(time_steps):
        # Update COM position based on tilt
        COM = positions[i, :]

        # Calculate tether properties
        _, _, teth_lengths = calculate_tether_vecs(COM, teth_anchor, offset)

        apex = calculate_apex(teth_lengths[0], teth_lengths[1], teth_anchor, offset)

        # update force at the start
        if i == 0:
            f = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_new = f
            f_old = f
            t1 = time_vec[i]
            j += 1
        # update force only at the prescribed update rate
        elif abs(time_vec[i] - update_vec[j]) < dt / 2:
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
        f_errors[i] = f_err
        ang_errors[i] = ang_err
        # torques[i] = np.linalg.norm(calculate_applied_torque(COM, tilt_angles[i, 0], tilt_angles[i, 1], f, r))

    # Plot results
    plot_simulation_results(time_vec, positions, tilt_angles, f_errors, ang_errors, torques, tether1_vec, tether2_vec)


if __name__ == "__main__":
    main()
