# this checks the error that comes from the tethers not being connected at one point and solving for the tether forces
# by translating each vector radially from the anchor point to the center until they touch. It then applies those tether
# forces to the original attachment vectors
# this model's major assumptions is that the waist is circular
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

    angle_err = np.acos(np.dot(expected_f_vec, f_vec)/(np.linalg.norm(expected_f_vec)*np.linalg.norm(f_vec)))*(180/np.pi)
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

    return f_err, angle_err


def main():
    mass = 200.0
    COM = np.array([0.0, 0.0, 3.0])
    # assuming waist circumference of 36 inches (3 feet), circular waist
    r = 3 / (2 * np.pi)
    print("actual center of mass: ", COM)

    teth_lengths = calculate_tether_length(COM, r)
    a = teth_lengths[0]
    b = teth_lengths[1]
    c = teth_lengths[2]

    apex = calculate_apex(a, b, c, r)
    print("tether convergence point: ",apex)
    f = calculate_tether_forces(apex, mass, r)

    f_err, angle_err = calculate_tether_error(COM, f, mass, r)

    print("force error (lb): ", f_err)
    print("angular error (deg): ", angle_err)


if __name__ == "__main__":
    main()
