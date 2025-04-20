import three_Tether_Dynamic_Model_eqns
import params
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os


def main():
    eqns = three_Tether_Dynamic_Model_eqns.three_teth_model()
    p = params.three_teth_Parameters()

    name = "100lbs_5-10deg_0.25Hz_PID"
    df = pd.read_csv(r"Trimmed_Dynamic_Testing_Data/TRIMMED_tether_data_20250416_100lbs_5-10deg_0.25Hz_PID.csv")

    # Define the folder path
    folder = 'plots/' + name
    # Create the folder if it doesn't exist
    os.makedirs(folder, exist_ok=True)
    folder_model = 'plots/model_' + name
    os.makedirs(folder_model, exist_ok=True)

    # sim time
    dt = 0.001

    # Create the time vector
    time = df['ResponseTime'] / 1000

    # Define your time range
    t_start = time.iloc[0]  # in seconds
    t_end = time.iloc[-1]  # in seconds
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
    f_err_act = df['ForceError_lbf_'][mask]
    ang_err_act = df['AngleError_deg_'][mask]
    Xapex = (df['XApex'] * 12)[mask]
    Yapex = (df['YApex'] * 12)[mask]
    Zapex = (df['ZApex'] * 12)[mask]

    # Original time vector from test data
    orig_time_vec = time.to_numpy()

    # Create a new time vector with 0.001 second intervals
    new_time_vec = np.arange(orig_time_vec[0], orig_time_vec[-1], dt)

    # Original position vector
    orig_apex_vec = np.column_stack(((df['XApex'])[mask], (df['YApex'])[mask], (df['ZApex'])[mask]))

    # Interpolate the position vector for the new time points
    x_interp = np.interp(new_time_vec, orig_time_vec, orig_apex_vec[:, 0])
    y_interp = np.interp(new_time_vec, orig_time_vec, orig_apex_vec[:, 1])
    z_interp = np.interp(new_time_vec, orig_time_vec, orig_apex_vec[:, 2])

    # Create the new interpolated apex vector
    apex_vec = np.column_stack((x_interp, y_interp, z_interp))

    # Use the new time vector for simulation
    time_vec = new_time_vec


    # linear assumption for time it takes servo to reach new torque
    reaction_t = 0.025  # seconds
    # parameters defining old force and time for linear convergence
    f = np.array([[0.0], [0.0], [0.0]])
    f_new = np.array([[0.0], [0.0], [0.0]])
    f_old = np.array([[0.0], [0.0], [0.0]])
    t1 = 0

    force_update_rate = round(dt + 0.025,4)
    update_vec = np.arange(time_vec[0], time_vec[-1]+force_update_rate, force_update_rate)
    # separate iterator for force update
    j = 0

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
        apex = apex_vec[i, :]

        # update force at the start
        if i == 0:
            f = eqns.calculate_tether_forces(apex)
            f_new = f
            f_old = f
            t1 = time_vec[i]
            j += 1
        # update force only at the prescribed update rate
        elif time_vec[i] >= update_vec[j]:
            # f = calculate_tether_forces(apex, mass, teth_anchor, offset)
            f_new = eqns.calculate_tether_forces(apex)
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


        # Calculate errors
        f_err, ang_err, tether1_vec[i], tether2_vec[i], tether3_vec[i] = eqns.calculate_tether_error(apex, f)

        # Store results
        f_errors[i] = f_err
        ang_errors[i] = ang_err

    # Plot results
    plot_simulation_results(time_vec, name, apex_vec, f_errors, ang_errors,
                            tether1_vec, tether2_vec, tether3_vec)

    # plot csv results
    plot_actual_results(time, name, teth1length, teth2length, teth3length, teth1ref_force, teth2ref_force,
                        teth3ref_force,
                        teth1loadcell_force, teth2loadcell_force, teth3loadcell_force, f_err_act, ang_err_act, Xapex, Yapex,
                        Zapex)

    plt.show()



def plot_simulation_results(time_vec, name, positions, f_errors, ang_errors,
                            tether1vec, tether2vec, tether3vec):
    figsize = (10, 4)

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
    fig1.savefig('plots/model_' + name + '/3Dtraj_' + name + '.png')


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
    fig2.savefig('plots/model_' + name + '/pos_' + name + '.png')

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
    fig3.savefig('plots/model_' + name + '/f_err_' + name + '.png')

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
    fig4.savefig('plots/model_' + name + '/ang_err_' + name + '.png')

    # Tether force component plots
    # Tether 1
    fig5 = plt.figure(figsize=figsize)
    ax5 = fig5.add_subplot(311)
    ax5.plot(time_vec, np.linalg.norm(tether1vec, axis=1), label='tether 1')
    ax5.set_ylabel('Tether 1 Force (lbf)')
    ax5.grid(True)
    ax5.legend()

    # Tether 2
    ax6 = fig5.add_subplot(312)
    ax6.plot(time_vec, np.linalg.norm(tether2vec, axis=1), label='tether 2')
    ax6.set_ylabel('Tether 2 Force (lbf)')
    ax6.grid(True)
    ax5.legend()

    # Tether 3
    ax7 = fig5.add_subplot(313)
    ax7.plot(time_vec, np.linalg.norm(tether3vec, axis=1), label='tether 3')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Tether 3 Force (lbf)')
    ax7.grid(True)
    ax5.legend()
    fig5.suptitle('Tether Force Vector magnitude Over Time')
    plt.tight_layout()
    fig5.savefig('plots/model_' + name + '/teth_forces_' + name + '.png')


    plt.tight_layout()


def plot_actual_results(time, name, teth1length, teth2length, teth3length, teth1ref_force, teth2ref_force, teth3ref_force,
                        teth1loadcell_force, teth2loadcell_force, teth3loadcell_force, f_err, ang_err, Xapex, Yapex,
                        Zapex):
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
    plt.savefig('plots/' + name + '/teth_lengths_' + name + '.png')

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

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
    plt.savefig('plots/' + name + '/tether_forces_subplot_' + name + '.png')

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
    plt.savefig('plots/' + name + '/apex_' + name + '.png')

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
    plt.savefig('plots/' + name + '/force_err_' + name + '.png')

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
    plt.savefig('plots/' + name + '/ang_err_' + name + '.png')







if __name__ == "__main__":
    main()