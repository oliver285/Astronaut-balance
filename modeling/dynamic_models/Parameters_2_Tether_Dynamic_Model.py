import numpy as np

class Parameters:
    mass = 100.0  # person's weight (lb)
    r = 2 / (2 * np.pi)  # waist radius (ft)
    waist_height = 2.5
    # mass = 200.0  # person's weight (lb)
    # r = 3 / (2 * np.pi)  # waist radius (ft)
    # waist_height = -3.1

    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other
    
    teth_anchor = [[2.0,                     0.0],
                   [-2.0,                     0.0]]
    offset = [[-r,                     0.0],
              [r,                     0.0]]

    dt = 0.002
    force_update_rate = dt + 0.005
    reaction_t = 0.023
