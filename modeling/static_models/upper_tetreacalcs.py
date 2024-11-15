# this checks the error that comes from the tethers not being connected at one point and solving for the tether forces
# at a point lower than the actual CG where the lengths converge. It then applies those tether forces to the original
# attachment vectors
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def equations(p, a, b, c):
    x, y, z = p
    return (
        x**2 + (y-2)**2 + z**2 - a**2,
        (x-2*np.cos(210*np.pi/180))**2 + (y-2*np.sin(210*np.pi/180))**2 + z**2 - b**2,
        (x-2*np.cos(330*np.pi/180))**2 + (y-2*np.sin(330*np.pi/180))**2 + z**2 - c**2
    )


def calculate_apex(a, b, c):
    initial_guess = [2, 2, 4]  # Start with a point above the base
    apex = fsolve(equations, initial_guess, args=(a, b, c))
    return apex

def calculate_tether_forces(apex, mass):
    teth1_vec = np.array(apex) - np.array([0, 2, 0])
    teth2_vec = np.array(apex) - np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0])
    teth3_vec = np.array(apex) - np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0])
    teth1_hat = teth1_vec/np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec/np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec/np.linalg.norm(teth3_vec)
    M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                   [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                   [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
    M2 = np.array([[0],[0],[mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f

def calculate_tether_length(COM,h):
    # must change r fir z direction to get upper then subtract height to get lower bounded

    #todo need to find a way to get new length height and subtract from assumed height

    teth1_attach_loc = np.array(COM) 
    teth2_attach_loc = np.array(COM) 
    teth3_attach_loc = np.array(COM)
    teth_lengths = np.array([0.0, 0.0, 0.0])
    # add Z portion to each lenngth
    teth_lengths[0] = np.linalg.norm(teth1_attach_loc - np.array([0.0, 2.0, 0.0]))
    teth_lengths[1] = np.linalg.norm(teth2_attach_loc - np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0]))
    teth_lengths[2] = np.linalg.norm(teth3_attach_loc - np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0]))
    return teth_lengths

def calculate_tether_length(COM,h):
    # must change r fir z direction to get upper then subtract height to get lower bounded

    #todo need to find a way to get new length height and subtract from assumed height

    teth1_attach_loc = np.array(COM) 
    teth2_attach_loc = np.array(COM) 
    teth3_attach_loc = np.array(COM)
    teth_lengths = np.array([0.0, 0.0, 0.0])
    # add Z portion to each lenngth
    teth_lengths[0] = np.linalg.norm(teth1_attach_loc - np.array([0.0, 2.0, 0.0]))
    teth_lengths[1] = np.linalg.norm(teth2_attach_loc - np.array([2*np.cos(210*np.pi/180), 2*np.sin(210*np.pi/180), 0]))
    teth_lengths[2] = np.linalg.norm(teth3_attach_loc - np.array([2*np.cos(330*np.pi/180), 2*np.sin(330*np.pi/180), 0]))
    return teth_lengths

def calculate_tether_error(COM, f, mass, h):
    # Calculate tether vectors
    tethvecs = -calculate_tether_length(COM, h)

    # Compute individual force vectors
    teth1_vec = f[0] * (tethvecs[0] / np.linalg.norm(tethvecs[0]))
    teth2_vec = f[1] * (tethvecs[1] / np.linalg.norm(tethvecs[1]))
    teth3_vec = f[2] * (tethvecs[2] / np.linalg.norm(tethvecs[2]))

    # Expected force vector (gravity acting downwards on the mass)
    expected_f_vec = np.array([0, 0, -mass])
    f_vec = teth1_vec + teth2_vec + teth3_vec

    # Calculate angle and force error
    angle_err = np.arccos(np.dot(expected_f_vec, f_vec) / (np.linalg.norm(expected_f_vec) * np.linalg.norm(f_vec))) * (180 / np.pi)
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

    return f_err, angle_err


def main():
    mass = 200.0
    COM = np.array([0.0, 0.0, 3.0])
    print("actual center of mass: ", COM)

    teth_lengths = calculate_tether_length(COM)
    a = teth_lengths[0]
    b = teth_lengths[1]
    c = teth_lengths[2]

    apex = calculate_apex(a, b, c)
    print("tether convergence point: ",apex)
    f = calculate_tether_forces(apex, mass)

    f_err, angle_err = calculate_tether_error(COM, f, mass)

    print("force error (lb): ", f_err)
    print("angular error (deg): ", angle_err)


if __name__ == "__main__":
    main()