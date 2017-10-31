#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: kinematic_constraint.py
Author: Stanislas Brossette
Email: stanislas.brossette@gmail.com
Github: https://github.com/stanislas-brossette
Description: In this file we try to figure out a linear cstr that represents conservatively a kinematic feasibility nonlinear constraint. We apply that on a toy problem with a kinematic arm that has 2 revolute joints, the entry variable of the problem is the location (x,z) of the hip, and the constraint is on the knee's location (which is a nonlinear function of the hip location computed via IK) that has to remain inside a given area (Square)
"""

import IK
import numpy as np
import collections
import matplotlib.pyplot as plt
from scipy.optimize import minimize

l1 = 0.5
l2 = 0.5

def main():
    xHip = np.array([[1,0],[0.5,0.5],[0.6,0.3],[0.3,0.8],[0.9, 0.0]])
    nSamples = len(xHip)
    cons = []
    cons = add_cstr_e_positive(cons)
    for i in range(nSamples):
        cons = add_cstr_feAb(cons, xHip[i,:])
    x0 = np.array([10,-20,10,-10, 0,10,3,0,3,0,0,0, 2,0,0,0])
    res = minimize(cost_norm_e, x0, jac=cost_norm_e_deriv, constraints=cons, method='SLSQP', options={'disp': False})
    sol = solutionStructure(res.x)
    print('e', sol.e)
    print('A', sol.A)
    print('b', sol.b)
    checkXHip(xHip, sol)

def concat(res,a):
    if len(res)==0:
        res = np.array([a])
    else:
        res = np.concatenate((res,np.array([a])))
    return res

# Checks if the value in xHip_vec are satisfactory for the nonlinear constraint and its approximation
def checkXHip(xHip_vec, solStruct):
    xHip_success_nonlinear = []
    xHip_success_linear = []
    xHip_failure_nonlinear = []
    xHip_failure_linear = []
    for i in range(len(xHip_vec[:,0])):
        currentXHip = xHip_vec[i,:]
        res_nonlinear = kin_cstr(currentXHip)
        if(np.all(res_nonlinear>=0)):
            xHip_success_nonlinear = concat(xHip_success_nonlinear, currentXHip)
        else:
            xHip_failure_nonlinear = concat(xHip_failure_nonlinear,currentXHip)

        res_linear = solStruct.A.dot(currentXHip) + solStruct.b
        if(np.all(res_linear>=0)):
            xHip_success_linear = concat(xHip_success_linear,currentXHip)
        else:
            xHip_failure_linear = concat(xHip_failure_linear,currentXHip)

    #reshape all the arrays for plot
    np.reshape(xHip_success_nonlinear,(len(xHip_success_nonlinear),2))
    np.reshape(xHip_success_linear,(len(xHip_success_linear),2))
    np.reshape(xHip_failure_nonlinear,(len(xHip_failure_nonlinear),2))
    np.reshape(xHip_failure_linear,(len(xHip_failure_linear),2))

    fig, (axNL, axL) = plt.subplots(1,2,sharex=True,sharey=True)
    axNL.set_xlim([-1,1])
    axNL.set_ylim([-1,1])
    axL.set_xlim([-1,1])
    axL.set_ylim([-1,1])
    #print('xHip_success_nonlinear', xHip_success_nonlinear)
    #print('xHip_success_linear', xHip_success_linear)
    #print('xHip_failure_nonlinear', xHip_failure_nonlinear)
    #print('xHip_failure_linear', xHip_failure_linear)
    if len(xHip_success_nonlinear)>0:
        axNL.plot(xHip_success_nonlinear[:,0],xHip_success_nonlinear[:,1],'xb')
    if len(xHip_failure_nonlinear)>0:
        axNL.plot(xHip_failure_nonlinear[:,0],xHip_failure_nonlinear[:,1],'xr')
    if len(xHip_success_linear)>0:
        axL.plot(xHip_success_linear[:,0],xHip_success_linear[:,1],'xb')
    if len(xHip_failure_linear)>0:
        axL.plot(xHip_failure_linear[:,0],xHip_failure_linear[:,1],'xr')
    plt.show()


def solutionStructure(x):
    solStruct = collections.namedtuple('solStruct', ['e', 'A', 'b'])
    e = x[0:4]
    A = np.zeros((4,2))
    A[:,0] = x[4:8]
    A[:,1] = x[8:12]
    b = x[12:16]
    res = solStruct(e,A,b)
    return res


#This function returns the nonlinear constraint that we want to approximate
def kin_cstr(xHip, xKneeMin=np.array([0.3,0.1]), xKneeMax=np.array([0.5,0.4]), opt_print=False):
    knee = IK.kneeFromHip(xHip)
    res = np.zeros(4)
    res[0] = xKneeMax[0]-knee[0]
    res[1] = xKneeMax[1]-knee[1]
    res[2] = -xKneeMin[0]+knee[0]
    res[3] = -xKneeMin[1]+knee[1]
    if(opt_print):
        print('KIN_CSTR:')
        print('knee:', knee)
        print('res:', res)
    return res

def kin_cstr_from_xKnee(knee, xKneeMin=np.array([0.3,0.1]), xKneeMax=np.array([0.5,0.4]), opt_print=False):
    res = np.zeros(4)
    res[0] = xKneeMax[0]-knee[0]
    res[1] = xKneeMax[1]-knee[1]
    res[2] = -xKneeMin[0]+knee[0]
    res[3] = -xKneeMin[1]+knee[1]
    if(opt_print):
        print('KIN_CSTR_FROM_XKNEE:')
        print('knee:', knee)
        print('res:', res)
    return res

def cost_norm_e(x):
    res = x[0] + x[1] + x[2] + x[3]
    return res

def cost_norm_e_deriv(x):
    res = np.array([1,1,1,1, 0,0,0,0,0,0,0,0, 0,0,0,0])
    return res

def cstr_e_positive(x):
    res = x[0:4]
    return res

def cstr_e_positive_deriv(x):
    res = np.zeros((4,16))
    res[0,0] = 1
    res[1,1] = 1
    res[2,2] = 1
    res[3,3] = 1
    return res

def add_cstr_e_positive(cons):
    cons.append({
            'type': 'ineq',
            'fun': lambda x: x[0],
            'jac': lambda x: np.array([1,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0])
        })
    cons.append({
            'type': 'ineq',
            'fun': lambda x: x[1],
            'jac': lambda x: np.array([0,1,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0])
        })
    cons.append({
            'type': 'ineq',
            'fun': lambda x: x[2],
            'jac': lambda x: np.array([0,0,1,0, 0,0,0,0,0,0,0,0, 0,0,0,0])
        })
    cons.append({
            'type': 'ineq',
            'fun': lambda x: x[3],
            'jac': lambda x: np.array([0,0,0,1, 0,0,0,0,0,0,0,0, 0,0,0,0])
        })
    return cons

def cstr_feAb_indiv(x, xHip, index, kin):
    data = solutionStructure(x)
    res = 0
    if(index < 4):
        res = - kin[index] + data.e[index] + data.A.dot(xHip)[index] + data.b[index]
    elif(index < 8):
        index = index - 4
        res = kin[index] - data.A.dot(xHip)[index] - data.b[index]
    return res

def cstr_feAb_indiv_deriv(x, xHip, index):
    data = solutionStructure(x)
    res = np.zeros(16)
    if(index < 4):
        res[index] = 1
        res[4+index] = xHip[0]
        res[8+index] = xHip[1]
        res[12+index] = 1
    elif(index < 8):
        index = index-4
        res[4+index] = -xHip[0]
        res[8+index] = -xHip[1]
        res[12+index] = -1
    return res

def add_cstr_feAb(cons,xHip):
    kin = kin_cstr(xHip)
    for index in range(8):
        cons.append({
            'type': 'ineq',
            'fun': lambda x: cstr_feAb_indiv(x, xHip, index, kin),
            'jac': lambda x: cstr_feAb_indiv_deriv(x, xHip, index)
            })
    return cons

#def cstr_feAb(x, xHip):
#    data = solutionStructure(x)
#    res = np.empty(8)
#    kin = kin_cstr(xHip)
#    res[0:4] = - kin + data.e + data.A.dot(xHip) + data.b
#    res[4:8] = kin - data.A.dot(xHip) - data.b
#    return res
#
#def cstr_feAb_deriv(x, xHip):
#    data = solutionStructure(x)
#    res = np.zeros((8,16))
#    res[0:4,0:4] = np.eye(4)
#    res[0:4,4:8] = xHip[0]*np.eye(4)
#    res[0:4,8:12] = xHip[1]*np.eye(4)
#    res[0:4,12:16] = np.eye(4)
#
#    res[4:8,4:8] = -xHip[0]*np.eye(4)
#    res[4:8,8:12] = -xHip[1]*np.eye(4)
#    res[4:8,12:16] = -np.eye(4)
#
#    return res


#main()
