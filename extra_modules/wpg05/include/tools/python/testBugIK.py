#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: testBugIK.py
Author: Stanislas Brossette
Email: stanislas.brossette@gmail.com
Github: https://github.com/stanislas-brossette
Description: 
"""

import IK as ik
import kinematic_constraint as kc
import numpy as np
import collections
import matplotlib.pyplot as plt
from scipy.optimize import minimize

q1Min = -1.0
q1Max = 1.0
q2Min = -2.0
q2Max = 0.0
dq1 = 0.2723735/7.0
dq2 = 0.11353/4.0
kneeXmin = 0.3
kneeXmax = 0.5
kneeYmin = 0.1
kneeYmax = 0.4

def plotIKvsGroundTruth(theta, thetaIK, xHipGoal):
    kin = ik.direct_kinematics(theta[0], theta[1])
    x = np.array([0.0, kin.knee[0], kin.hip[0]])
    y = np.array([0.0, kin.knee[1], kin.hip[1]])
    plt.plot(x, y)

    kinIK = ik.direct_kinematics(thetaIK[0], thetaIK[1])
    xIK = np.array([0.0, kinIK.knee[0], kinIK.hip[0]])
    yIK = np.array([0.0, kinIK.knee[1], kinIK.hip[1]])
    plt.plot(xIK, yIK)

    plt.plot(xHipGoal[0], xHipGoal[1], 'rx')

    plt.show()

def main():
    solutions = []
    solutionsIK = []
    failures = []
    failuresIK = []
    for q1 in np.arange(q1Min, q1Max, dq1):
        print('q1', q1)
        for q2 in np.arange(q2Min, q2Max, dq2):
            kin = ik.direct_kinematics(q1, q2)

            resCstrKnee = kc.kin_cstr_from_xKnee(kin.knee, np.array([kneeXmin, kneeYmin]), np.array([kneeXmax, kneeYmax]))
            if np.all(resCstrKnee>=0):
                solutions.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})
            else:
                failures.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})

            resCstrIK = kc.kin_cstr(kin.hip, np.array([kneeXmin, kneeYmin]), np.array([kneeXmax, kneeYmax]))
            if np.all(resCstrIK>=0):
                solutionsIK.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})
            else:
                failuresIK.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})

            resIK = ik.inverse_kinematics(kin.hip) 

            if(np.linalg.norm(np.array([q1,q2])-resIK)>1e-2):
                print('\nq1:', q1)
                print('q1IK:', resIK[0])
                print('q2:', q2)
                print('q2IK:', resIK[1])
                plotIKvsGroundTruth(np.array([q1,q2]),resIK,kin.hip)



    
    solHipMat = np.zeros((len(solutions), 2))
    solKneeMat = np.zeros((len(solutions), 2))
    for i in range(len(solutions)):
        solHipMat[i,:] = solutions[i]['hip']
        solKneeMat[i,:] = solutions[i]['knee']
    failHipMat = np.zeros((len(failures), 2))
    failKneeMat = np.zeros((len(failures), 2))
    for i in range(len(failures)):
        failHipMat[i,:] = failures[i]['hip']
        failKneeMat[i,:] = failures[i]['knee']

    solHipMatIK = np.zeros((len(solutionsIK), 2))
    solKneeMatIK = np.zeros((len(solutionsIK), 2))
    for i in range(len(solutionsIK)):
        solHipMatIK[i,:] = solutionsIK[i]['hip']
        solKneeMatIK[i,:] = solutionsIK[i]['knee']
    failHipMatIK = np.zeros((len(failuresIK), 2))
    failKneeMatIK = np.zeros((len(failuresIK), 2))
    for i in range(len(failuresIK)):
        failHipMatIK[i,:] = failuresIK[i]['hip']
        failKneeMatIK[i,:] = failuresIK[i]['knee']

    fig, (axIK, axKnee) = plt.subplots(1,2,sharex=True,sharey=True)
    axKnee.plot(failHipMat[:,0], failHipMat[:,1], '.r')
    axKnee.plot(failKneeMat[:,0], failKneeMat[:,1], 'xr')
    axKnee.plot(solHipMat[:,0], solHipMat[:,1], '.b')
    axKnee.plot(solKneeMat[:,0], solKneeMat[:,1], 'xb')
    axKnee.set_title('Ground truth')
    axIK.plot(failHipMatIK[:,0], failHipMatIK[:,1], '.r')
    axIK.plot(failKneeMatIK[:,0], failKneeMatIK[:,1], 'xr')
    axIK.plot(solHipMatIK[:,0], solHipMatIK[:,1], '.b')
    axIK.plot(solKneeMatIK[:,0], solKneeMatIK[:,1], 'xb')
    axIK.set_title('Solutions with IK')

    axIK.set_xlim(-1.0,1.0)
    axKnee.set_xlim(-1.0,1.0)

    axIK.axis('equal')
    axKnee.axis('equal')
    
    plt.show()
    

main()

