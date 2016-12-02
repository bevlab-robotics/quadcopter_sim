#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import geometry_msgs
import tf
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm

class quadcopter:

    def __init__(self):
        self.R = np.eye(3) # rotation matrix from body to inertial frames
        self.p = np.zeros((3,1)) # translation in inertial frame
        self.m = 0.5 # quadcoper mass in kg
        self.I = np.eye(3) # Inertia matrix
        self.w = np.zeros((3,1)) # angular velocity in body frame
        self.v = np.zeros((3,1)) # translational velocity in body frame
        self.f = np.zeros((3,1)) # External force in inertial frame
        self.tau = np.zeros((3,1)) # External moment in body frame
        self.rccommand = rccommand() # Contains roll, pitch, yaw, and throttle rc commands

    def sim(self, dt):
        # simulates using Newton-Euler equations
        # dt is step time in seconds

        # Determine state derivatives
        pdot = self.v # time derivative of position
        vdot = 1/self.m*self.f # time derivative of velocity
        Rdot = self.R.dot(skew(self.w).dot(self.R.T))  # time derivative of rotation matrix
        # Rdot = R*S(w)*R^T
        wdot = np.linalg.inv(self.I).dot(self.tau - skew(self.w).dot(self.I).dot(self.w))   # time derivative of angular velocity
        #  tau  = I*wdot + S(w)*I*w
        #  wdot = inv(I)*(tau-S(w)*I*w)

        # Forward Euler method - integrate state derivatives one time step
        self.p += dt*pdot
        self.v += dt*vdot
        self.w += dt*wdot
        # For rotation matrix use exponential map - geodesic in SO(3)
        self.R = self.R.dot(expm(dt*self.R.dot(skew(self.w)))) #
        # R = R*expm(dt*R*S(w))

    def rccommand2tau(self):
        #
        tau = 0.0
        return tau



class rccommand:
    def __init__(self):
        self.roll = 0.0 # Roll command
        self.pitch = 0.0 # Pitch RC command
        self.yaw = 0.0 # Yaw RC command
        self.throttle = 0.0 # Throttle RC command

def skew(w):
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]
    return np.array([[0,-w3,w2],[w3,0,-w1],[-w2,w1,0]])

if __name__ == '__main__':
    q = quadcopter()
    q.tau = np.array([[0],[0],[1]])
    q.f = np.array([[1],[0],[0]])
    dt = 0.1 # time step
    for i in range(10):
        q.sim(dt)
        print q.R
        print q.p




