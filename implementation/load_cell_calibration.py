# load cell 2
meas = [1.38, 47.42]

ref = [0.0, 45.2]


slope = (meas[1]-meas[0])/(ref[1]-ref[0])


scale = 1/slope


print(scale)