#! /usr/bin/env python
import numpy as np

def qv_mult(q1, v1):
    q2 = np.insert(v1, 3, 0.0)
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[0:3]

def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.asarray([x, y, z, w])

def q_conjugate(q):
    x, y, z, w = q
    return np.asarray([-x, -y, -z, w])