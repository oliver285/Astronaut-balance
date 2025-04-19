import three_Tether_Dynamic_Model_eqns
import params
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    eqns = three_Tether_Dynamic_Model_eqns.three_teth_model()
    p = params.three_teth_Parameters()

    # test data
    df = pd.read_csv(r"TRIMMED_100lbs_Xoscillations_5deg_0.25Hz_PI.csv")

    # Simulation parameters
    mass = p.mass  # person's weight (lb)
    initial_height = p.waist_height
    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other
    teth_anchor = p.teth_anchor
    # attachment offset vector 3x3 each row is the vector for each tether
    # assuming circular radius (pointing from tether attachment loc to COM
    offset = p.offset


    # todo: time now from test data
    time_vec = df['ResponseTime'].to_numpy()/1000

    # linear assumption for time it takes servo to reach new torque
    reaction_t = 0.33  # seconds
    # parameters defining old force and time for linear convergence
    f = np.array([[0.0], [0.0], [0.0]])
    f_new = np.array([[0.0], [0.0], [0.0]])
    f_old = np.array([[0.0], [0.0], [0.0]])
    t1 = 0

    # todo: position now from test data
    apex_vec = np.column_stack((df['XApex'], df['YApex'], df['ZApex']))

    # Initialize arrays to store results
    f_errors = np.zeros(len(time_vec))
    ang_errors = np.zeros(len(time_vec))

    # Arrays to store tether force vectors
    tether1_vec = np.zeros((len(time_vec), 3))
    tether2_vec = np.zeros((len(time_vec), 3))
    tether3_vec = np.zeros((len(time_vec), 3))

    f_old = eqns.calculate_tether_forces(apex_vec[0, :])
    # Run simulation
    for i in range(len(time_vec)):
        # todo: from data
        apex = apex_vec[i, :]

        f_new = eqns.calculate_tether_forces(apex)
        t1 = time_vec[i]

        # if torque is not reached yet
        if (time_vec[i] < t1 + reaction_t):
            f = ((f_new - f_old) / reaction_t) * (time_vec[i]) - ((f_new - f_old) / reaction_t) * t1 + f_old
        # if we reach torque before next update
        else:
            f = ((f_new - f_old) / reaction_t) * (t1 + reaction_t) - ((f_new - f_old) / reaction_t) * t1 + f_old

        f_old = f

        # Calculate errors
        f_err, ang_err, tether1_vec[i], tether2_vec[i], tether3_vec[i] = eqns.calculate_tether_error(apex, f)

        # Store results
        f_errors[i] = f_err
        ang_errors[i] = ang_err

    # Plot results
    plot_simulation_results(time_vec, apex_vec, f_errors, ang_errors,
                            tether1_vec, tether2_vec, tether3_vec)

    plt.show()


def plot_simulation_results(time_vec, positions, f_errors, ang_errors,
                            tether1vec, tether2vec, tether3vec):
    # Set figure size and style for all plots
    plt.style.use('default')
    figsize = (8, 4)

    p = params.three_teth_Parameters()

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


    # Position components plot
    fig2 = plt.figure(figsize=figsize)
    ax2 = fig2.add_subplot(111)
    ax2.plot(time_vec, positions[:, 0], label='X')
    ax2.plot(time_vec, positions[:, 1], label='Y')
    ax2.plot(time_vec, positions[:, 2], label='Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (ft)')
    ax2.set_title(f'Position Components Over Time')
    ax2.grid(True)
    ax2.legend()
    ax2.set_ylim([-5, 5])
    plt.tight_layout()
    # fig3.savefig(f'plots_report/3_tether_position_{tilt_axis}_mass_{p.mass}.png')

    # Force error plot
    fig3 = plt.figure(figsize=figsize)
    ax3 = fig3.add_subplot(111)
    ax3.plot(time_vec, f_errors)
    ax3.axhline(y=5, color='r', linestyle=':', label='Max Allowable Error (5 lbf)', linewidth=3)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Force Error (lbf)')
    ax3.set_title(f'Tether Force Error')
    ax3.grid(True)
    ax3.legend()
    # ax3.set_ylim([0, 8])
    plt.tight_layout()
    # fig4.savefig(f'plots_report/3_tether_force_error_{tilt_axis}_mass_{p.mass}.png')

    # Angular error plot
    fig4 = plt.figure(figsize=figsize)
    ax4 = fig4.add_subplot(111)
    ax4.plot(time_vec, ang_errors)
    ax4.axhline(y=2, color='r', linestyle=':', label='Max Allowable Error (2 deg)', linewidth=3)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angular Error (deg)')
    ax4.set_title(f'Tether Angular Error')
    ax4.grid(True)
    ax4.legend()
    # ax4.set_ylim([0, 4])
    plt.tight_layout()
    # fig5.savefig(f'plots_report/3_tether_angular_error_{tilt_axis}_mass_{p.mass}.png')

    # Tether force component plots
    # Tether 1
    fig5 = plt.figure(figsize=figsize)
    ax5 = fig5.add_subplot(311)
    ax5.plot(time_vec, tether1vec[:, 0], label='X')
    ax5.plot(time_vec, tether1vec[:, 1], label='Y')
    ax5.plot(time_vec, tether1vec[:, 2], label='Z')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Tether 1 Force Components (lbf)')
    ax5.grid(True)
    ax5.legend()
    ax5.set_ylim([-200, 200])

    # Tether 2
    ax6 = fig5.add_subplot(312)
    ax6.plot(time_vec, tether2vec[:, 0], label='X')
    ax6.plot(time_vec, tether2vec[:, 1], label='Y')
    ax6.plot(time_vec, tether2vec[:, 2], label='Z')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Tether 2 Force Components (lbf)')
    ax6.grid(True)
    ax6.set_ylim([-200, 200])

    # Tether 3
    ax7 = fig5.add_subplot(313)
    ax7.plot(time_vec, tether3vec[:, 0], label='X')
    ax7.plot(time_vec, tether3vec[:, 1], label='Y')
    ax7.plot(time_vec, tether3vec[:, 2], label='Z')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Tether 3 Force Components (lbf)')
    ax7.grid(True)
    ax7.set_ylim([-200, 200])
    fig5.suptitle('Tether Force Vector Components Over Time')
    plt.tight_layout()


    plt.tight_layout()

if __name__ == "__main__":
    main()