import numpy as np

class two_Teth_Parameters:
    # mass = 100.0  # person's weight (lb)
    # r = 2 / (2 * np.pi)  # waist radius (ft)
    # waist_height = -2.5
    mass = 200.0  # person's weight (lb)
    r = 6.0/12.0  # waist radius (ft)
    waist_height = -3.1

    # tether anchor loc 3x3 each row is the vector for each tether
    # assuming anchor locations are radially 2 feet away from person 120 degrees away from each other

    teth_anchor = [[2.0, 0.0],
                   [-2.0, 0.0]]
    offset = [[-r, 0.0],
              [r, 0.0]]


class three_teth_Parameters:
    mass = 100.0  # person's weight (lb)
    r = 6.0/12.0  # waist radius (ft)
    waist_height = -2.5
    # mass = 200.0  # person's weight (lb)
    # r = 3 / (2 * np.pi)  # waist radius (ft)
    # waist_height = -3.1

    # tether anchor loc 3x3 each row is the vector for each tether
    # teth_anchor = [[-2,                      0.0,                     0.0],
    #               [2.0, -2.0, 0.0],
    #               [2.0, 2.0, 0.0]]

    teth_anchor = [[-22.0/12.0, 0.5625/12.0, -11.125/12.0],
                   [22.0/12.0, -27.0625/12.0, -11.125/12.0],
                   [22.0/12.0, 27.0625/12.0, -11.125/12.0]]

    offset = [[r,                      0.0,                     0.0],
              [-r*np.cos(315*np.pi/180), -r*np.sin(315*np.pi/180), 0.0],
              [-r*np.cos(45*np.pi/180), -r*np.sin(45*np.pi/180), 0.0]]
