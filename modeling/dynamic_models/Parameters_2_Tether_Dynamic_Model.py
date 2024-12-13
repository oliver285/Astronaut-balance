import numpy as np

class Parameters:
    mass = 200.0  # person's weight (lb)
    r = 3 / (2 * np.pi)  # waist radius (ft)
    waist_height = 3.1
    teth_percent_error = 0.05  # percent error in tether length

    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other
    
    teth_anchor = [[2.0,                     0.0],
                   [-2.0,                     0.0]]
    offset = [[-r,                     0.0],
              [r,                     0.0]]
              
    dt = 0.002
    force_update_rate = 0.004
    reaction_t = 0.01
