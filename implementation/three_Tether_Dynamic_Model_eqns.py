import numpy as np
from scipy.optimize import fsolve
from params import three_teth_Parameters

class three_teth_model:
    def __init__(self):
        p = three_teth_Parameters()
        self.mass = p.mass
        self.r = p.r
        self.waist_height = p.waist_height
        self.teth_anchor = p.teth_anchor
        self.offset = p.offset

    def equations(self, p, a, b, c):
        x, y, z = p
        return (
            # front tether
            (x - (self.teth_anchor[0][0] + self.offset[0][0])) ** 2 + (y - (self.teth_anchor[0][1] + self.offset[0][1])) ** 2 + (
                        z - (self.teth_anchor[0][2] + self.offset[0][2])) ** 2 - a ** 2,

            # left tether
            (x - (self.teth_anchor[1][0] + self.offset[1][0])) ** 2 + (y - (self.teth_anchor[1][1] + self.offset[1][1])) ** 2 + (
                        z - (self.teth_anchor[1][2] + self.offset[1][2])) ** 2 - b ** 2,

            # right tether
            (x - (self.teth_anchor[2][0] + self.offset[2][0])) ** 2 + (y - (self.teth_anchor[2][1] + self.offset[2][1])) ** 2 + (
                        z - (self.teth_anchor[2][2] + self.offset[2][2])) ** 2 - c ** 2,
        )

    def calculate_apex(self, a, b, c):
        initial_guess = [2, 2, -4]  # Start with a point above the base
        apex = fsolve(self.equations, np.array(initial_guess), args=(a, b, c))
        return apex

    def calculate_tether_vecs(self, COM):
        # determine the tether unit vector
        teth1_vec = np.array(self.teth_anchor[0][:]) - (np.array(COM) - np.array(self.offset[0][:]))
        teth2_vec = np.array(self.teth_anchor[1][:]) - (np.array(COM) - np.array(self.offset[1][:]))
        teth3_vec = np.array(self.teth_anchor[2][:]) - (np.array(COM) - np.array(self.offset[2][:]))
        teth1_hat = teth1_vec / np.linalg.norm(teth1_vec)
        teth2_hat = teth2_vec / np.linalg.norm(teth2_vec)
        teth3_hat = teth3_vec / np.linalg.norm(teth3_vec)
        lengths = np.array([np.linalg.norm(teth1_vec), np.linalg.norm(teth2_vec), np.linalg.norm(teth3_vec)])

        return (teth1_hat, teth2_hat, teth3_hat, lengths, teth1_vec, teth2_vec, teth3_vec)

    def calculate_tether_forces(self, apex):
        # solve the system of eqns of the unit vectors to find the equations
        teth1_hat, teth2_hat, teth3_hat, _, _, _, _ = self.calculate_tether_vecs(apex)

        M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                       [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                       [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
        M2 = np.array([[0], [0], [self.mass]])
        f = np.dot(np.linalg.inv(M1), M2)
        return f

    def calculate_tether_error(self, COM, f):
        teth1_hat, teth2_hat, teth3_hat, _, _, _, _ = self.calculate_tether_vecs(COM)
        teth1_vec = f[0] * teth1_hat
        teth2_vec = f[1] * teth2_hat
        teth3_vec = f[2] * teth3_hat

        expected_f_vec = np.array([0, 0, self.mass])
        f_vec = teth1_vec + teth2_vec + teth3_vec

        # find the angle between expected and actual
        angle_err = np.arccos(
            np.dot(expected_f_vec, f_vec) / (np.linalg.norm(expected_f_vec) * np.linalg.norm(f_vec))) * (180 / np.pi)
        # determine the difference between their forces
        f_err = np.abs(np.linalg.norm(expected_f_vec) - np.linalg.norm(f_vec))

        return f_err, angle_err, teth1_vec, teth2_vec, teth3_vec
