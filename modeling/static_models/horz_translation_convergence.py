# this checks the error that comes from the tethers not being connected at one point and solving for the tether forces
# by translating each vector radially from the anchor point to the center until they touch. It then applies those tether
# forces to the original attachment vectors
# this model's major assumptions is that the waist is circular, the tether connection triad on the user is always
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
    temp = fsolve(equations, initial_guess, args=(a, b, c, r))
    apex = [[temp[0]], [temp[1]], [temp[2]]]
    return apex


def calculate_tether_forces(apex, mass, r):
    teth1_vec = np.array(apex) - np.array([[0], [(2-r)], [0]])
    teth2_vec = np.array(apex) - np.array([[(2-r)*np.cos(210*np.pi/180)], [(2-r)*np.sin(210*np.pi/180)], [0]])
    teth3_vec = np.array(apex) - np.array([[(2-r)*np.cos(330*np.pi/180)], [(2-r)*np.sin(330*np.pi/180)], [0]])
    teth1_hat = teth1_vec/np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec/np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec/np.linalg.norm(teth3_vec)
    M1 = np.array([[teth1_hat[0][0], teth2_hat[0][0], teth3_hat[0][0]],
                   [teth1_hat[1][0], teth2_hat[1][0], teth3_hat[1][0]],
                   [teth1_hat[2][0], teth2_hat[2][0], teth3_hat[2][0]]])
    M2 = np.array([[0],[0],[mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f


# this is in base frame
def calculate_tether_length(COM, r):
    teth1_attach_loc = np.array(COM) + np.array([[0.0], [r], [0.0]])
    teth2_attach_loc = np.array(COM) + np.array([[r*np.cos(210*np.pi/180)], [r*np.sin(210*np.pi/180)], [0]])
    teth3_attach_loc = np.array(COM) + np.array([[r*np.cos(330*np.pi/180)], [r*np.sin(330*np.pi/180)], [0]])
    teth_lengths = np.array([[0.0], [0.0], [0.0]])
    teth_lengths[0] = np.linalg.norm(teth1_attach_loc - np.array([[0.0], [2.0], [0.0]]))
    teth_lengths[1] = np.linalg.norm(teth2_attach_loc - np.array([[2*np.cos(210*np.pi/180)], [2*np.sin(210*np.pi/180)], [0]]))
    teth_lengths[2] = np.linalg.norm(teth3_attach_loc - np.array([[2*np.cos(330*np.pi/180)], [2*np.sin(330*np.pi/180)], [0]]))
    return teth_lengths

def calculate_applied_torque(COM, theta, psi, f, r):
    # _bf signifies body frame
    teth1_attach_loc_bf = np.array(COM) + np.array([[0.0], [r], [0.0]])
    teth2_attach_loc_bf = np.array(COM) + np.array([[r*np.cos(210*np.pi/180)], [r*np.sin(210*np.pi/180)], [0]])
    teth3_attach_loc_bf = np.array(COM) + np.array([[r*np.cos(330*np.pi/180)], [r*np.sin(330*np.pi/180)], [0]])
    # _basef signifies base frame
    r2 = np.array([[np.cos(theta), 0,  -np.sin(theta)],
                   [0,             1,  0],
                   [np.sin(theta), 0,  np.cos(theta)]])
    r3 = np.array([[np.cos(psi),  np.sin(psi), 0],
                   [-np.sin(psi), np.cos(psi), 0],
                   [0,            0,           1]])
    # rotate z to align y-axis, rotate y-axis and rotate z again to get to base frame
    rot = np.linalg.inv(r3) @ np.linalg.inv(r2) @ r3
    teth1_attach_loc_basef = rot @ teth1_attach_loc_bf
    teth2_attach_loc_basef = rot @ teth2_attach_loc_bf
    teth3_attach_loc_basef = rot @ teth3_attach_loc_bf

    teth1_init_vec = np.array([[0.0], [2.0], [0.0]]) - teth1_attach_loc_basef
    teth2_init_vec = np.array([[2*np.cos(210*np.pi/180)], [2*np.sin(210*np.pi/180)], [0]]) - teth2_attach_loc_basef
    teth3_init_vec = np.array([[2*np.cos(330*np.pi/180)], [2*np.sin(330*np.pi/180)], [0]]) - teth3_attach_loc_basef
    teth1_vec = f[0][0]*(teth1_init_vec/np.linalg.norm(teth1_init_vec))
    teth2_vec = f[1][0]*(teth2_init_vec/np.linalg.norm(teth2_init_vec))
    teth3_vec = f[2][0]*(teth3_init_vec/np.linalg.norm(teth3_init_vec))
    teth1_vec = teth1_vec[:, 0]
    teth2_vec = teth2_vec[:, 0]
    teth3_vec = teth3_vec[:, 0]

    COM_basef = rot @ COM

    r1_vec = teth1_attach_loc_basef - COM_basef
    r2_vec = teth2_attach_loc_basef - COM_basef
    r3_vec = teth3_attach_loc_basef - COM_basef
    r1_vec = r1_vec[:, 0]
    r2_vec = r2_vec[:, 0]
    r3_vec = r3_vec[:, 0]

    torque1_vec = np.cross(r1_vec, teth1_vec)
    torque2_vec = np.cross(r2_vec, teth2_vec)
    torque3_vec = np.cross(r3_vec, teth3_vec)

    torque_vec = torque1_vec + torque2_vec + torque3_vec

    return torque_vec

# hip rotation incorporated here for "true" hip movement
def calculate_tether_error(COM, theta, psi, f, mass, r):
    # _bf signifies body frame
    teth1_attach_loc_bf = np.array(COM) + np.array([[0.0], [r], [0.0]])
    teth2_attach_loc_bf = np.array(COM) + np.array([[r*np.cos(210*np.pi/180)], [r*np.sin(210*np.pi/180)], [0]])
    teth3_attach_loc_bf = np.array(COM) + np.array([[r*np.cos(330*np.pi/180)], [r*np.sin(330*np.pi/180)], [0]])
    # _basef signifies base frame
    r2 = np.array([[np.cos(theta), 0,  -np.sin(theta)],
                   [0,             1,  0],
                   [np.sin(theta), 0,  np.cos(theta)]])
    r3 = np.array([[np.cos(psi),  np.sin(psi), 0],
                   [-np.sin(psi), np.cos(psi), 0],
                   [0,            0,           1]])
    # rotate z to align y-axis, rotate y-axis and rotate z again to get to base frame
    rot = np.linalg.inv(r3) @ np.linalg.inv(r2) @ r3
    teth1_attach_loc_basef = rot @ teth1_attach_loc_bf
    teth2_attach_loc_basef = rot @ teth2_attach_loc_bf
    teth3_attach_loc_basef = rot @ teth3_attach_loc_bf

    teth1_init_vec = np.array([[0.0], [2.0], [0.0]]) - teth1_attach_loc_basef
    teth2_init_vec = np.array([[2*np.cos(210*np.pi/180)], [2*np.sin(210*np.pi/180)], [0]]) - teth2_attach_loc_basef
    teth3_init_vec = np.array([[2*np.cos(330*np.pi/180)], [2*np.sin(330*np.pi/180)], [0]]) - teth3_attach_loc_basef
    teth1_vec = f[0][0]*(teth1_init_vec/np.linalg.norm(teth1_init_vec))
    teth2_vec = f[1][0]*(teth2_init_vec/np.linalg.norm(teth2_init_vec))
    teth3_vec = f[2][0]*(teth3_init_vec/np.linalg.norm(teth3_init_vec))

    expected_f_vec = np.array([[0], [0], [-mass]])
    f_vec = teth1_vec + teth2_vec + teth3_vec
    expected_f_vec = expected_f_vec[:, 0]
    f_vec = f_vec[:, 0]

    angle_err = np.acos(np.dot(expected_f_vec, f_vec)/(np.linalg.norm(expected_f_vec)*np.linalg.norm(f_vec)))*(180/np.pi)
    f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

    return f_err, angle_err


def plot_error(psi_vec, f_err, ang_err):
    psi_vec = np.rad2deg(psi_vec)

    plt.figure(1)
    plt.subplot(2,1,1)
    plt.plot(psi_vec, f_err)
    plt.ylabel("tether vec force error (lbf)")
    plt.title("tether error vs changes in rotation about center of platform")
    plt.subplot(2, 1, 2)
    plt.plot(psi_vec, ang_err)
    plt.xlabel("yaw rotation about center of platform (deg)")
    plt.ylabel("tether vec angle error (deg)")

    plt.show()

def plot_torque(psi_vec, torque):
    psi_vec = np.rad2deg(psi_vec)
    plt.figure(2)
    plt.plot(psi_vec, torque)
    plt.ylabel("torque applied (lb*ft)")
    plt.title("torque applied vs changes in rotation about center of platform")

    plt.show()

def main():
    # persons weight (lb)
    mass = 200.0
    # center of mass vec
    person_h = 3.0
    # assuming waist circumference of 36 inches (3 feet), circular waist
    r = 3 / (2 * np.pi)
    # center of mass in body frame will always be this
    COM_bf = np.array([[0.0], [0.0], [person_h]])
    # y-rot
    theta = np.deg2rad(10.)
    # z-rot
    # psi = np.deg2rad(0.)
    psi_vec = np.deg2rad(np.linspace(0, 360, 100))

    f_err = np.zeros(100)
    ang_err = np.zeros(100)
    torque_vec = np.zeros((3, 100))
    torque = np.zeros(100)
    for (psi, i) in zip(psi_vec, range(100)):
        # rotation matrix using the euler angles to get our base frame COM
        r2 = np.array([[np.cos(theta), 0, -np.sin(theta)],
                       [0, 1, 0],
                       [np.sin(theta), 0, np.cos(theta)]])
        r3 = np.array([[np.cos(psi), np.sin(psi), 0],
                       [-np.sin(psi), np.cos(psi), 0],
                       [0, 0, 1]])
        # rotate z to align y-axis, rotate y-axis and rotate z again to get to base frame
        rot = np.linalg.inv(r3) @ np.linalg.inv(r2) @ r3
        COM_basef = rot @ COM_bf
        # print("actual center of mass: ", COM_basef)

        teth_lengths = calculate_tether_length(COM_basef, r)
        a = teth_lengths[0][0]
        b = teth_lengths[1][0]
        c = teth_lengths[2][0]

        apex = calculate_apex(a, b, c, r)
        # print("tether convergence point: ", apex)
        f = calculate_tether_forces(apex, mass, r)

        f_err[i], ang_err[i] = calculate_tether_error(COM_bf, theta, psi, f, mass, r)
        torque_vec[:, i] = calculate_applied_torque(COM_bf, theta, psi, f, r)
        torque[i] = np.linalg.norm(torque_vec[:, i])


    plot_error(psi_vec, f_err, ang_err)
    plot_torque(psi_vec, torque)
    # print("force error (lb): ", f_err)
    # print("angular error (deg): ", ang_err)
    print(torque)




if __name__ == "__main__":
    main()
