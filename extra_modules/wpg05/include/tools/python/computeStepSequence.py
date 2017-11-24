#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Compute a sequence of steps for a given speed, width etc...

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import argparse
import collections

np.set_printoptions(threshold=100000000000000)

#Default values
default_CoM0 = [0.0, 0.0, 0.0]
default_step_duration = 0.8
default_step_width = 0.2
default_walking_speed = [0.2, 0.0, 0.0]
default_number_of_steps = 10
default_time_step = 0.1

#Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("--CoM0", help="Initial position of the CoM", nargs=3,
        type=float, default=default_CoM0)
parser.add_argument("--step_width", help="Step width",
        type=float, default=default_step_width)
parser.add_argument("--step_duration", help="Step duration",
        type=float, default=default_step_duration)
parser.add_argument("--time_step", help="Time step",
        type=float, default=default_time_step)
parser.add_argument("--speed", help="Walking speed", nargs=3,
        type=float, default=default_walking_speed)
parser.add_argument("--number_of_steps", help="Number of steps in the simulation",
        type=int, default=default_number_of_steps)
parser.add_argument("--plot", help="If True, the constraint polygon is plotted",
        type=bool, default='False')
args = parser.parse_args()

stepConfiguration = collections.namedtuple('stepConfiguration', ['position', 'tMin', 'tMax'])

def main(args):
    CoM0 = np.array(args.CoM0)
    step_width = args.step_width
    step_duration = args.step_duration
    speed = np.array(args.speed)
    number_of_steps = args.number_of_steps
    step_length = step_duration*speed
    time_step = args.time_step

    rightFootSteps = [stepConfiguration([CoM0[0], CoM0[1]-step_width/2.0, 0.0], 0.0, 2*step_duration-0.01)]
    current_time = 3*step_duration
    leftFootSteps = [stepConfiguration([CoM0[0], CoM0[1]+step_width/2.0, 0.0], 0.0, current_time-0.01)] 
    yVec = np.array([0,1,0])
    for i in np.arange(number_of_steps):
        latestLeftStep = leftFootSteps[len(leftFootSteps)-1]
        newRightStep = stepConfiguration(latestLeftStep.position + step_length - step_width*yVec, current_time - time_step + 0.01, current_time + step_duration -0.01)
        newLeftStep = stepConfiguration(newRightStep.position + step_length + step_width*yVec , current_time + step_duration - time_step + 0.01, current_time + 2*step_duration -0.01)
        rightFootSteps.append(newRightStep)
        leftFootSteps.append(newLeftStep)
        current_time = current_time + 2*step_duration

    lastLeftStep = leftFootSteps[len(leftFootSteps)-1]
    leftFootSteps[len(leftFootSteps)-1] = stepConfiguration(lastLeftStep.position, lastLeftStep.tMin, 100.0)
    lastRightStep = rightFootSteps[len(rightFootSteps)-1]
    rightFootSteps[len(rightFootSteps)-1] = stepConfiguration(lastRightStep.position, lastRightStep.tMin, 100.0)

    printStepPlan(rightFootSteps, leftFootSteps)

    return

def printStep(stepConfig):
    print("    - [", stepConfig.position[0], ", ", stepConfig.position[1], ", ", stepConfig.position[2], ", ", stepConfig.tMin, ", ", stepConfig.tMax, "]")

def printStepPlan(rightFootSteps, leftFootSteps):
    print("  rightStepsParameters:")
    for rStep in rightFootSteps:
        printStep(rStep)
    print("  leftStepsParameters:")
    for lStep in leftFootSteps:
        printStep(lStep)


main(args)

