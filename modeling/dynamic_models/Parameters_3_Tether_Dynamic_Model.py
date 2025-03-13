import numpy as np

class Parameters:
    mass = 200.0  # person's weight (lb)
    r = 3 / (2 * np.pi)  # waist radius (ft)
    # waist_height = -2.5
    # mass = 200.0  # person's weight (lb)
    # r = 3 / (2 * np.pi)  # waist radius (ft)
    waist_height = -3.0

    # tether anchor loc 3x3 each row is the vector for each tether
    teth_anchor = [[-2,                      0.0,                     0.0],
                  [2.0, -2.0, 0.0],
                  [2.0, 2.0, 0.0]]
    offset = [[r,                      0.0,                     0.0],
              [-r*np.cos(315*np.pi/180), -r*np.sin(315*np.pi/180), 0.0],
              [-r*np.cos(45*np.pi/180), -r*np.sin(45*np.pi/180), 0.0]]

    dt = 0.002
    force_update_rate = dt + 0.005
    reaction_t = 0.023
