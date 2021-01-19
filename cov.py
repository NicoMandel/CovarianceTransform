#!/usr/bin/env python3

"""
    File to test the conversion from quaternion covariances to euler angles
"""
import numpy as np

def convert(quat, quatcov):
    """
        Function to do the conversion
    """
    jac = jacobian(quat)
    eul_cov = jac @ quatcov @ jac.T
    return eul_cov

def jacobian(quat):
    """
        Function to return the jacobian - as defined by:
        https://stats.stackexchange.com/questions/119780/what-does-the-covariance-of-a-quaternion-mean
    """
    jac = np.zeros((3,4))
    q32n = quat[2] - quat[1]
    q32p = quat[2] + quat[1]
    q41p = quat[3] + quat[0]
    q41n = quat[3] - quat[0]
    normalizerpos = q32p**2 + q41p**2
    normalizerneg = q32n**2 + q41n**2

    q41pos = q41p / normalizerpos
    q32pos = q32p / normalizerpos
    q32neg = q32n / normalizerneg
    q41neg = q41n / normalizerneg
    
    # Row 1 of the jacobian
    psi1 = (q32pos*-1.0) + (q32neg)
    psi2 = (q41pos) - (q41neg)
    psi3 = (q41pos) + (q41neg)
    psi4 = (q32pos*-1.0) - (q32neg)

    # Row 3 of the Jacobian
    phi1 = psi4
    phi2 = psi3
    phi3 = psi2
    phi4 = psi1

    # Row 2 of the Jacobian
    theta_norm = 2 / np.sqrt(1 - 4 * ((quat[1] * quat[2] + quat[0] * quat[3])**2)) 

    theta1 = theta_norm * quat[3]
    theta2 = theta_norm * quat[2]
    theta3 = theta_norm * quat[1]
    theta4 = theta_norm * quat[0]

    # filling in the jacobian
    jac[0,0] = psi1
    jac[0,1] = psi2
    jac[0,2] = psi3
    jac[0,3] = psi4

    jac[1,0] = theta1
    jac[1,1] = theta2
    jac[1,2] = theta3
    jac[1,3] = theta4

    jac[2,0] = phi1
    jac[2,1] = phi2
    jac[2,2] = phi3
    jac[2,3] = phi4

    return jac

def randomQuat():
    """
        Following The definition used by Eigen
        Linked here: http://planning.cs.uiuc.edu/node198.html
    """
    unitquat = np.zeros(4)
    u = np.random.rand(3)
    unitquat[0] = np.sqrt(1-u[0]) * np.sin(2*np.pi*u[1])
    unitquat[1] = np.sqrt(1-u[0]) * np.cos(2*np.pi*u[1])
    unitquat[2] = np.sqrt(u[0]) * np.sin(2*np.pi*u[2])
    unitquat[3] = np.sqrt(u[0]) * np.cos(2*np.pi*u[2])
    unitquat = unitquat / np.linalg.norm(unitquat)
    return unitquat


if __name__=="__main__":
    # seed the random generator
    np.random.seed(1)

    # dummy covariance
    quatcov = np.ones((4,4))

    quatlist = []
    for i in range(1000):
        quat = randomQuat()
        eucov = convert(quat, quatcov)
    print("completed all random test cases. Proceeding")
    
    quat = np.ones(4) / np.linalg.norm(np.ones(4))
    print("Attempting numerically instable version: {}".format(quat))

    eucov = convert(quat, quatcov)
    print(eucov)

    

    