#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: IK.py
Author: Stanislas Brossette
Email: stanislas.brossette@gmail.com
Github: https://github.com/stanislas-brossette
Description: Necessary tools to compute IK
"""

import numpy as np
import collections
import matplotlib.pyplot as plt
from scipy.optimize import minimize

l1 = 0.5
l2 = 0.5

#This function is used to check the derivatives by finite difference
def test_finite_diff(theta1, theta2):
    delta = 1e-8
    xHip = np.array([0.5,0.1])

    print('\ntest Direct Kinematics')
    dDKdthetaFD = np.zeros((2,2))
    dDKdthetaFD[:,0] = (direct_kinematics(theta1+delta,theta2).hip-direct_kinematics(theta1,theta2).hip)/delta
    dDKdthetaFD[:,1] = (direct_kinematics(theta1,theta2+delta).hip-direct_kinematics(theta1,theta2).hip)/delta
    print('dDKdthetaFD', dDKdthetaFD)
    jac = jacobian(theta1, theta2).hip
    print('dDKdthetaCalc', jac)

    print('\nTest Cost Function') 
    theta = np.array([theta1, theta2])
    thetaP1 = np.array([theta1+delta, theta2])
    thetaP2 = np.array([theta1, theta2+delta])
    dfdthetaFD = np.zeros(2)
    dfdthetaFD[0] = (cost_func_IK(thetaP1,xHip)-cost_func_IK(theta,xHip))/delta
    dfdthetaFD[1] = (cost_func_IK(thetaP2,xHip)-cost_func_IK(theta,xHip))/delta
    print('dfdthetaFD', dfdthetaFD)
    dfdthetaCalc = cost_func_IK_deriv(theta, xHip)
    print('dfdthetaCalc', dfdthetaCalc, '\n')

#Computes the direct kinematics from theta (returns knee and hip positions)
def direct_kinematics(theta1, theta2):
    x_knee = np.array([l1*np.cos(theta1), l1*np.sin(theta1)])
    x_hip = x_knee + np.array([l2*np.cos(theta1 + theta2), l2*np.sin(theta1 + theta2)])
    robotKinematics = collections.namedtuple('positions', ['knee', 'hip'])
    res = robotKinematics(x_knee, x_hip)
    return res

#Computes the jacobians of the knee and hip positions
def jacobian(theta1, theta2):
    jac_hip = np.matrix([[-l1*np.sin(theta1) - l2*np.sin(theta1 + theta2), -l2*np.sin(theta1 + theta2)],
                         [+l1*np.cos(theta1) + l2*np.cos(theta1 + theta2), +l2*np.cos(theta1 + theta2) ]])
    jac_knee = np.matrix([[-l1*np.sin(theta1) , 0],
                          [+l1*np.cos(theta1), 0]])
    robotJacobians = collections.namedtuple('jacobians', ['knee', 'hip'])
    jac = robotJacobians(jac_knee, jac_hip)
    return jac

def cstr_direct_kinematics(q, xHipGoal):
    x_knee = np.array([l1*np.cos(q[0]), l1*np.sin(q[0])])
    x_hip = x_knee + np.array([l2*np.cos(q[0] + q[1]), l2*np.sin(q[0] + q[1])])
    return x_hip-xHipGoal

def cstr_direct_kinematics_jacobian(q):
    jac_hip = np.matrix([[-l1*np.sin(q[0]) - l2*np.sin(q[0] + q[1]), -l2*np.sin(q[0] + q[1])],
                         [+l1*np.cos(q[0]) + l2*np.cos(q[0] + q[1]), +l2*np.cos(q[0] + q[1]) ]])
    return jac_hip



#Cost function measuring the squared distance to the target hip position
def cost_func_IK(theta, xHip):
    kin = direct_kinematics(theta[0], theta[1])
    res = (kin.hip[0]-xHip[0])**2 + (kin.hip[1]-xHip[1])**2
    return res

#Derivatives of the cost function
def cost_func_IK_deriv(theta, xHip):
    kin = direct_kinematics(theta[0], theta[1])
    jac = jacobian(theta[0], theta[1])
    dfdtheta0 = 2*(kin.hip[0]-xHip[0])*jac.hip[0,0] + 2*(kin.hip[1]-xHip[1])*jac.hip[1,0]
    dfdtheta1 = 2*(kin.hip[0]-xHip[0])*jac.hip[0,1] + 2*(kin.hip[1]-xHip[1])*jac.hip[1,1]
    return np.array([dfdtheta0, dfdtheta1])

    
#Computes thetas to reach the xHip goal with a simple minimization. Bounds cstr could be added
def inverse_kinematics(xHip, thetaInit=np.array([0.13,-0.15]), bndsTheta1=(-1.0,1.0), bndsTheta2=(-2.0,0.0)):
    #print('CALL IK: xHip:', xHip, 'thetaInit:', thetaInit, 'bndsTheta1:', bndsTheta1, 'bndsTheta2:', bndsTheta2)
    angles = collections.namedtuple('angles', ['theta1', 'theta2'])
    bnds = (bndsTheta1,bndsTheta2)
    cons = ({
        'type': 'eq',
        'fun': lambda x: cstr_direct_kinematics(x,xHip)[0],
        'jac': lambda x: cstr_direct_kinematics_jacobian(x)[0,:]
        },
        {
        'type': 'eq',
        'fun': lambda x: cstr_direct_kinematics(x,xHip)[1],
        'jac': lambda x: cstr_direct_kinematics_jacobian(x)[1,:]
        })
    #res = minimize(cost_func_IK, thetaInit, args=(xHip), jac=cost_func_IK_deriv, constraints=cons, method='SLSQP', options={'disp':False})
    res = minimize(cost_func_IK, thetaInit, args=(xHip), jac=cost_func_IK_deriv, bounds=bnds, method='SLSQP', options={'disp':False}, tol=1e-12)
    return res.x

#Funny plot of trajectories of the robot
def plot():
    nPoints = 1000
    vTheta1 = np.linspace(0,2*np.pi,nPoints)
    vTheta2 = np.linspace(0,10*np.pi,nPoints)
    vx_knee = np.empty(nPoints)
    vy_knee = np.empty(nPoints)
    vx_hip = np.empty(nPoints)
    vy_hip = np.empty(nPoints)
    for i in np.arange(0,nPoints):
      kin = direct_kinematics(vTheta1[i], vTheta2[i])
      vx_knee[i] = kin.knee[0]
      vy_knee[i] = kin.knee[1]
      vx_hip[i] = kin.hip[0]
      vy_hip[i] = kin.hip[1]
    plt.plot(vx_knee, vy_knee)
    plt.plot(vx_hip, vy_hip)
    plt.show()

#Plots the robot and target
def plot(theta, xHipGoal):
    kin = direct_kinematics(theta[0], theta[1])
    x = np.array([0.0, kin.knee[0], kin.hip[0]])
    y = np.array([0.0, kin.knee[1], kin.hip[1]])
    plt.plot(x, y)
    plt.plot(xHipGoal[0], xHipGoal[1], 'rx')
    plt.show()

#Returns the location of the knee for a given hip position
def kneeFromHip(xHip, thetaInit=np.array([0.13,-0.15]), bndsTheta1=(-1.0,1.0), bndsTheta2=(-2.0,0.0)):
    theta = inverse_kinematics(xHip, thetaInit, bndsTheta1, bndsTheta2)
    kin = direct_kinematics(theta[0], theta[1])
    return kin.knee
