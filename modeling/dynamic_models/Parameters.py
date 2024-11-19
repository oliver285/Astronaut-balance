import numpy as np

class Parameters:
    mass = 200.0  # person's weight (lb)
    r = 3 / (2 * np.pi)  # waist radius (ft)
    teth_percent_error = 0.05  # percent error in tether length

    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other
    # teth_anchor = [[2.0,                      0.0,                     0.0],
    #               [2.0*np.cos(240*np.pi/180), 2.0*np.sin(240*np.pi/180), 0.0],
    #               [2.0*np.cos(120*np.pi/180), 2.0*np.sin(120*np.pi/180), 0.0]]
    teth_anchor = [[2.0,                      0.0,                     0.0],
                  [2.0*np.cos(225*np.pi/180), 2.0*np.sin(225*np.pi/180), 0.0],
                  [2.0*np.cos(135*np.pi/180), 2.0*np.sin(135*np.pi/180), 0.0]]
    # attachment offset vector 3x3 each row is the vector for each tether
    # assuming circular radius (pointing from tether attachment loc to COM
    offset = [[-r,                      0.0,                     0.0],
              [-r*np.cos(225*np.pi/180), -r*np.sin(225*np.pi/180), 0.0],
              [-r*np.cos(135*np.pi/180), -r*np.sin(135*np.pi/180), 0.0]]
