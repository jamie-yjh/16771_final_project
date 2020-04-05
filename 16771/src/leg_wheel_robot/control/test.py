#!/usr/bin/env python3
# license removed for brevity
from scipy.spatial.transform import Rotation as R
import numpy as np
from control import lqr
h = 0.8
d = 0.2
a = 5/2
r = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])

a = r.as_euler('xyz', degrees=False)
print(a)
def state_space_params():
    
    A = np.array([[0,1,0,0,0,0],
        [38.4,0,0,0,0,0],
        [0,0,0,1,0,0],
        [-3,0,0,0,0,0],
        [0,0,0,0,0,1],
        [0,0,0,0,0,0]])
    B = np.array([[0,0],
        [-0.2449,0],
        [0,0],
        [0.0816,0],
        [0,0],
        [0,0.5]])
    Q = np.eye(6)
    R = np.eye(2)
    K,S,E = lqr(A,B,Q,R)
    print(K)

    return K

state_space_params()
print(2**2)