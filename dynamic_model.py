# Necessary imports
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time  # Import the time module





# Function to reverse engineer lengths from a given apex point
def reverse_engineer_lengths(apex):
    x, y, z = apex
    # Define the fixed points for the tether attachment locations
    point_a = np.array([0, 2, 0])
    point_b = np.array([2 * np.cos(210 * np.pi / 180), 2 * np.sin(210 * np.pi / 180), 0])
    point_c = np.array([2 * np.cos(330 * np.pi / 180), 2 * np.sin(330 * np.pi / 180), 0])
    
    # Convert apex into a numpy array for vectorized distance calculation
    apex_point = np.array([x, y, z])
    
    # Calculate lengths as distances from the apex to each fixed point
    a = np.linalg.norm(apex_point - point_a)
    b = np.linalg.norm(apex_point - point_b)
    c = np.linalg.norm(apex_point - point_c)
    
    return a, b, c


def calculate_tether_length(COM):
    # assuming waist circumference of 36 inches (3 feet), circular waist
    # todo: zero for now
    r = 3/(2*np.pi)
    teth1_attach_loc = np.array(COM) + np.array([0.0, r, 0.0])
    teth2_attach_loc = np.array(COM) + np.array([r*np.cos(210*np.pi/180), r*np.sin(210*np.pi/180), 0])
    teth3_attach_loc = np.array(COM) + np.array([r*np.cos(330*np.pi/180), r*np.sin(330*np.pi/180), 0])
    teth_lengths = np.array([0.0, 0.0, 0.0])
    teth_lengths[0] = np.linalg.norm(teth1_attach_loc - np.array([0.0, 2.0, 0.0]))
    teth_lengths[1] = np.linalg.norm(teth2_attach_loc - np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0]))
    teth_lengths[2] = np.linalg.norm(teth3_attach_loc - np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0]))
    return teth_lengths

# System of equations for the tether attachment points
def equations(p, a, b, c):
    x, y, z = p
    return (
        x**2 + (y-2)**2 + z**2 - a**2,
        (x-2*np.cos(210*np.pi/180))**2 + (y-2*np.sin(210*np.pi/180))**2 + z**2 - b**2,
        (x-2*np.cos(330*np.pi/180))**2 + (y-2*np.sin(330*np.pi/180))**2 + z**2 - c**2
    )

# Function to calculate the apex point based on tether lengths
def calculate_apex(a, b, c):
    initial_guess = [2, 2, 4]
    apex = fsolve(equations, initial_guess, args=(a, b, c))
    return apex

# Function to calculate tether forces at the apex
def calculate_tether_forces(apex, mass):
    # Define tether attachment points
    teth1_vec = np.array(apex) - np.array([0, 2, 0])
    teth2_vec = np.array(apex) - np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0])
    teth3_vec = np.array(apex) - np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0])
    
    # Calculate unit vectors
    teth1_hat = teth1_vec / np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec / np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec / np.linalg.norm(teth3_vec)
    
    # Set up matrix for force calculation
    M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                   [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                   [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
    M2 = np.array([[0], [0], [mass]])
    
    # Solve for forces in each tether
    f = np.dot(np.linalg.inv(M1), M2)
    return f.flatten()

# Function to calculate force and angle errors based on calculated tether forces
def calculate_tether_error(COM, f, mass):
    r = 3 / (2 * np.pi)
    # Define attachment points relative to COM
    teth1_attach_loc = np.array(COM) + np.array([0.0, r, 0.0])
    teth2_attach_loc = np.array(COM) + np.array([r * np.cos(210 * np.pi / 180), r * np.sin(210 * np.pi / 180), 0])
    teth3_attach_loc = np.array(COM) + np.array([r * np.cos(330 * np.pi / 180), r * np.sin(330 * np.pi / 180), 0])
    
    # Calculate initial tether vectors
    teth1_init_vec = np.array([0.0, 2.0, 0.0]) - teth1_attach_loc
    teth2_init_vec = np.array([2 * np.cos(210 * np.pi / 180), 2 * np.sin(210 * np.pi / 180), 0]) - teth2_attach_loc
    teth3_init_vec = np.array([2 * np.cos(330 * np.pi / 180), 2 * np.sin(330 * np.pi / 180), 0]) - teth3_attach_loc
    
    # Calculate actual force vectors
    teth1_vec = f[0] * (teth1_init_vec / np.linalg.norm(teth1_init_vec))
    teth2_vec = f[1] * (teth2_init_vec / np.linalg.norm(teth2_init_vec))
    teth3_vec = f[2] * (teth3_init_vec / np.linalg.norm(teth3_init_vec))
    
    # Calculate expected force vector and errors
    expected_f_vec = np.array([0, 0, -mass])
    f_vec = teth1_vec + teth2_vec + teth3_vec
    angle_err = np.degrees(np.arccos(np.dot(expected_f_vec, f_vec) / (np.linalg.norm(expected_f_vec) * np.linalg.norm(f_vec))))
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))
    
    return f_err, angle_err

# Adjust tether lengths iteratively to reduce errors
def adjust_tether_lengths(mass, initial_lengths, COM):
    max_force_error = 20
    max_angle_error = 5
    lengths = np.array(initial_lengths)
    adjustment_step = 0.05  # Initial adjustment step size
    iteration = 0

    while (iteration!=1000):
        # Calculate the apex based on the current lengths
        start_time = time.time()
        apex = calculate_apex(*lengths)
        
        # Calculate tether forces at this apex
        forces = calculate_tether_forces(apex, mass)
        
        # Calculate errors
        force_error, angle_error = calculate_tether_error(COM, forces, mass)

        # Print current state for debugging
       # print(f"Iteration {iteration}: Lengths: {lengths}, Force Error: {force_error}, Angle Error: {angle_error}")

        # Check if errors are within acceptable range
        if force_error < max_force_error and angle_error < max_angle_error:
            elapsed_time = time.time() - start_time
            print(f"Converged after {iteration} iterations")
            print(f"Final lengths: {lengths}")
            print(f"Final apex: {apex}")
            print(f"Final tether forces: {forces}")
            print(f"Final force error: {force_error}, angle error: {angle_error}")
            print(f"Time taken: {elapsed_time:.6f} seconds")
            break  # Exit loop if errors are within thresholds

        # Adjust each length in `lengths` based on errors

        if angle_error >= max_angle_error:
            lengths += adjustment_step * (angle_error / 100)  # Adjust lengths for angle error reduction

        # Update the adjustment step if needed
        if force_error < max_force_error / 2 and angle_error < max_angle_error / 2:
            adjustment_step *= 0.9  # Reduce step size as we approach convergence

        iteration += 1  # Increment iteration count

    return lengths, apex, forces, force_error, angle_error


# Main function to orchestrate calculations and results
# Main function to orchestrate calculations and results
def main():
    mass = 200.0
    theta = np.linspace(0, np.pi * 2, 50)  # Rotation angles for full rotation
    COM = np.array([0.0, 0.0, 3.0])
    max_force_error = 20
    max_angle_error = 5
    
    # Initialize lists to store results
    time_vector_const_torque = []
    time_vector_adjust_torque = []
    a_int, b_int, c_int = [], [], []
    apex, f, f_err, angle_err = [], [], [], []

    for i in range(50):
        # Start timing this iteration
        start_time = time.time()

        # Back-calculate lengths from the apex point
        input_apex = np.array([2 * np.cos(theta[i]), 2 * np.sin(theta[i]), 3])
        lengths = calculate_tether_length(input_apex)

        # Calculate the new apex from these lengths
        apex_val = calculate_apex(*lengths)
        apex.append(apex_val)

        # Calculate tether forces for the calculated apex
        f_val = calculate_tether_forces(apex_val, mass)
        f.append(f_val)

        # Calculate force and angle errors
        f_err_val, angle_err_val = calculate_tether_error(COM, f_val, mass)
        f_err.append(f_err_val)
        angle_err.append(angle_err_val)

        # End timing this iteration and store the time
        elapsed_time = time.time() - start_time
        time_vector_const_torque.append(elapsed_time)

        # Print iteration details
        print(f"Iteration {i}:")
        print(f"  Apex: {apex_val}")
        print(f"  Tether Forces: {f_val}")
        print(f"  Force Error: {f_err_val}, Angle Error: {angle_err_val}")
        print(f"  Time taken for calculation: {elapsed_time:.6f} seconds")

        # If errors exceed thresholds, adjust lengths and recalculate
        if f_err_val > max_force_error or angle_err_val > max_angle_error:
            # Call adjust_tether_lengths to bring errors within acceptable range
            lengths, adjusted_apex, adjusted_forces, adjusted_f_err, adjusted_angle_err = adjust_tether_lengths(mass, lengths, input_apex)
            
            # Measure and store the adjustment time
            adjustment_time = time.time() - start_time - elapsed_time
            time_vector_adjust_torque.append(adjustment_time)

            # Store results from adjustment
            apex.append(adjusted_apex)
            f.append(adjusted_forces)
            f_err.append(adjusted_f_err)
            angle_err.append(adjusted_angle_err)

            print(f"Adjusted Tether Lengths: {lengths}")
            print(f"  Adjusted Apex: {adjusted_apex}")
            print(f"  Adjusted Tether Forces: {adjusted_forces}")
            print(f"  Adjusted Force Error: {adjusted_f_err}, Adjusted Angle Error: {adjusted_angle_err}")
            print(f"  Time taken for adjustment: {adjustment_time:.6f} seconds")

    print("Completed all iterations and adjustments.")

if __name__ == "__main__":
    main()
