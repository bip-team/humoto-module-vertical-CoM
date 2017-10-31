#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: plotSolution.py
Author: Stanislas Brossette
Email: stanislas.brossette@gmail.com
Github: https://github.com/stanislas-brossette
Description: plots the solution area for the problem of finding xHip such that xKnee is in a square
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
    plt.xlim(-1.0,1.0)

    plt.axis('equal')

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
            cstrKneePassed = np.all(resCstrKnee>=0)
            if cstrKneePassed:
                solutions.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})
            else:
                failures.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})

            resCstrIK = kc.kin_cstr(kin.hip, np.array([kneeXmin, kneeYmin]), np.array([kneeXmax, kneeYmax]))
            cstrIKPassed = np.all(resCstrIK>=0)
            if cstrIKPassed:
                solutionsIK.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})
            else:
                failuresIK.append({'q1': q1, 'q2': q2, 'hip': kin.hip, 'knee': kin.knee})


            if(cstrKneePassed != cstrIKPassed):
                print('\nProblem')
                kc.kin_cstr_from_xKnee(kin.knee, np.array([kneeXmin, kneeYmin]), np.array([kneeXmax, kneeYmax]), True)
                kc.kin_cstr(kin.hip, np.array([kneeXmin, kneeYmin]), np.array([kneeXmax, kneeYmax]), True)
                resIK = ik.inverse_kinematics(kin.hip) 
                kinIK = ik.direct_kinematics(resIK[0], resIK[1])
                print('cstrKneePassed:', cstrKneePassed)
                print('cstrIKPassed:', cstrIKPassed)
                print('q1:', q1)
                print('q1IK:', resIK[0])
                print('q2:', q2)
                print('q2IK:', resIK[1])
                print('kin.hip', kin.hip)
                print('kinIK.hip', kinIK.hip)
                print('kin.knee', kin.knee)
                print('kinIK.knee', kinIK.knee)
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

    fig, axes = plt.subplots(2,2,sharex=True,sharey=True)
    axes[0,0].plot(failHipMat[:,0], failHipMat[:,1], '.r')
    axes[0,0].plot(failKneeMat[:,0], failKneeMat[:,1], 'xr')
    axes[1,0].plot(solHipMat[:,0], solHipMat[:,1], '.b')
    axes[1,0].plot(solKneeMat[:,0], solKneeMat[:,1], 'xb')
    axes[0,0].set_title('Ground truth Fail')
    axes[1,0].set_title('Ground truth Succ')
    axes[0,1].plot(failHipMatIK[:,0], failHipMatIK[:,1], '.r')
    axes[0,1].plot(failKneeMatIK[:,0], failKneeMatIK[:,1], 'xr')
    axes[1,1].plot(solHipMatIK[:,0], solHipMatIK[:,1], '.b')
    axes[1,1].plot(solKneeMatIK[:,0], solKneeMatIK[:,1], 'xb')
    axes[1,1].set_title('Solutions with IK Success')
    axes[0,1].set_title('Solutions with IK Fail')

    axes[1,1].set_xlim(-1.0,1.0)
    axes[1,0].set_xlim(-1.0,1.0)
    axes[0,1].set_xlim(-1.0,1.0)
    axes[0,0].set_xlim(-1.0,1.0)

    axes[1,1].axis('equal')
    axes[1,0].axis('equal')
    axes[0,1].axis('equal')
    axes[0,0].axis('equal')
    
    plt.show()
    

main()

