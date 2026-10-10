import numpy as np

def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_3r(angles, link_lengths):
    t1, t2, t3 = angles
    l1, l2, l3 = link_lengths

    T01 = dh_matrix(t1, 0, l1, 0)
    T12 = dh_matrix(t2, 0, l2, 0)
    T23 = dh_matrix(t3, 0, l3, 0)

    T02 = T01 @ T12
    T03 = T02 @ T23

    p0 = np.array([0, 0])
    p1 = T01[0:2, 3]
    p2 = T02[0:2, 3]
    p3 = T03[0:2, 3]

    return np.array([p0, p1, p2, p3])
