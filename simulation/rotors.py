import numpy as np

kf = 1e-6 # thrust coefficient
kd = 1e-8 # drag coefficient
l = 0.15 # arm length, meteres

w_max = 500 # rad/s

rotor_map = np.array([[kf, kf, kf, kf],
                      [kf, -kf, -kf, kf],
                      [kf, -kf, kf, -kf],
                      [-kf, -kf, kf, kf]]) # from omega^2 to thrust and torque

rotor_map_inv = np.linalg.inv(rotor_map) # from thrust and torque to omega^2


