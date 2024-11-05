# Necessary imports
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

# Main function to orchestrate calculations and results
def main():
    mass = 200.0
    theta = np.linspace(0, 360, 50)
    COM = np.array([0.0, 0.0, 3.0])
    
    # Initialize lists to store results
    time_vector_const_torque = []
    time_vector_adjust_torque = []
    a_int, b_int, c_int = [], [], []
    apex, f, f_err, angle_err = [], [], [], []

    for i in range(0, 50):
        # Back-calculate lengths from the apex point
        input_apex = np.array([2*np.cos(np.radians(theta[i])),2, 4])
        lengths = reverse_engineer_lengths(input_apex)
        
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

        print(f"Iteration {i}:")
        print(f"  Apex: {apex_val}")
        print(f"  Tether Forces: {f_val}")
        print(f"  Force Error: {f_err_val}, Angle Error: {angle_err_val}")

if __name__ == "__main__":
    main()
