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

# Function to calculate strain
def straincalcs(tau, L0):
    # Constants
    r = 1  # radius in inches of Motor
    A = 1  # area (update as needed) for tether
    E = 1  # modulus of tether

    # Calculate force
    F = tau / r
    stress = F / A
    epsilon = stress / E
    Delta_L = L0 * epsilon

    return Delta_L


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

def main():
    mass = 200.0
    tau1 = np.linspace(0, 10, 50)  # lbs
    tau2 = np.linspace(0, 10, 50)
    tau3 = np.linspace(0, 10, 50)
    COM = np.array([0.0, 0.0, 3.0])
    L01 = 1
    L02 = 1
    L03 = 1

    # Initialize empty lists to store results
    a = []
    b = []
    c = []
    apex = []
    f = []

    for i in range(0, 10, 50):  # Adjust range if needed for your specific logic

        # Calculate strains based on tau values and L0
        a_val = straincalcs(tau1[i], L01)
        b_val = straincalcs(tau2[i], L02)
        c_val = straincalcs(tau3[i], L03)
        
        # Append calculated values to lists
        a.append(a_val)
        b.append(b_val)
        c.append(c_val)

        # Calculate apex for the current strain values
        apex_val = calculate_apex(a_val, b_val, c_val)
        apex.append(apex_val)
        print(f"Tether convergence point at index {i}: ", apex_val)

        # Calculate tether forces for the current apex
        f_val = calculate_tether_forces(apex_val, mass)
        f.append(f_val)
        print(f"Tether forces at index {i}: ", f_val)
    L01 = a_val
    L02 = b_val
    L03 = c_val


if __name__ == "__main__":
    main()
