#!/usr/bin/env python3

"""
    File to find the edge cases where the condition is given
    for unit quaternions.
        function theta_norm is the one that throws the error
        theta_norm = 2 / np.sqrt(1 - 4 * ((quat[1] * quat[2] + quat[0] * quat[3])**2)) 
        Find all values where theta_norm will throw a divide by 0 error!
        Also, the other denominators might throw errors       
"""

from .cov import randomQuat
import numpy as np


if __name__=="__main__":
    pass