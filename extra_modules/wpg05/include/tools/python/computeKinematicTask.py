#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import argparse
import collections

np.set_printoptions(threshold=100000000000000)

#Default values
default_height = 0.5
default_nIter = 3
default_radius = 1.0
default_center = [0.0, 0.0, 0.0]

#Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("--height", help="Height of the half-space",
        type=float, default=default_height)
parser.add_argument("--nIter", help="Number of iteration of the spheres discretization",
        type=int, default=default_nIter)
parser.add_argument("--radius", help="Radius of the sphere",
        type=float, default=default_radius)
parser.add_argument("--center", help="Center of the sphere", nargs=3,
        type=float, default=default_center)
parser.add_argument("--fileName", help="Name of the file where to write the constraint",
        type=str, default='../../wpg05/task_kinematic.h.in')
parser.add_argument("--plot", help="If True, the constraint polygon is plotted",
        type=bool, default='False')
args = parser.parse_args()

flatPolygon = collections.namedtuple('flatPolygon', ['vertices', 'normal', 'center'])
cstr = collections.namedtuple('cstr', ['A', 'b'])

def main(args):
    center = np.array(args.center)
    nIter = args.nIter
    radius = args.radius
    height = args.height

    faces = discretizeSphere(center, radius, nIter)
    spherePolygons = computeNormals(faces, center)
    constraintPolygons = cutSphere(spherePolygons, height, center)
    constraint = polygonToMatrixConstraint(constraintPolygons)
    writeConstraintToFile(constraint, args)

    if args.plot:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax = addFacesToPlot(ax, constraintPolygons)
        ax = addNormalsToPlot(ax, constraintPolygons)
        ax = addRandomPointsCheck(ax, constraintPolygons, 1000)
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-1.0, 1.0)
        plt.show()

    return

def writeConstraintToFile(constraint, args):
    A = constraint.A
    b = constraint.b
    myFile = open(args.fileName, 'w')
    myFile.write('#pragma once\n')
    myFile.write('#include <Eigen/Core>\n\n')
    myFile.write('namespace humoto\n')
    myFile.write('{\n')
    myFile.write('namespace wpg05\n')
    myFile.write('{\n')
    myFile.write('class GeneratedKinematicConstraint\n')
    myFile.write('{\n')
    myFile.write('  public:\n')
    myFile.write('    double height_;\n')
    myFile.write('    double radius_;\n')
    myFile.write('    int nIter_;\n')
    myFile.write('    Eigen::Vector3d center_;\n')
    myFile.write('    Eigen::MatrixXd A;\n')
    myFile.write('    Eigen::VectorXd b;\n')
    myFile.write('    GeneratedKinematicConstraint()\n')
    myFile.write('    {\n')
    myFile.write('      height_ = ')
    myFile.write(str(args.height))
    myFile.write(';\n      nIter_ = ')
    myFile.write(str(args.nIter))
    myFile.write(';\n      radius_ = ')
    myFile.write(str(args.radius))
    myFile.write(';\n      center_ = Eigen::Vector3d(')
    myFile.write(str(args.center[0]))
    myFile.write(', ')
    myFile.write(str(args.center[1]))
    myFile.write(', ')
    myFile.write(str(args.center[2]))
    myFile.write(');\n      A.resize(')
    myFile.write(str(A.shape[0]))
    myFile.write(',')
    myFile.write(str(A.shape[1]))
    myFile.write(');\n')
    myFile.write('      b.resize(')
    myFile.write(str(b.shape[0]))
    myFile.write(');\n')
    myFile.write('      A << ')
    for i in np.arange(A.shape[0]):
        for j in np.arange(A.shape[1]):
            myFile.write(str(A[i,j]))
            if not (i == A.shape[0]-1 and j == A.shape[1]-1):
                myFile.write(', ')
    myFile.write(';\n')
    myFile.write('      b << ')
    for i in np.arange(b.shape[0]):
        myFile.write(str(b[i,0]))
        if i != b.shape[0]-1:
            myFile.write(', ')
    myFile.write(';\n')
    myFile.write('    }\n};\n}\n}\n')

#Computes A and b such that b<Ax if the x is inside the polytope
def polygonToMatrixConstraint(polygon):
    A = np.zeros((len(polygon),3))
    b = np.zeros((len(polygon),1))
    for i in np.arange(len(polygon)):
        A[i,:] = polygon[i].normal
        b[i,0] = np.dot(polygon[i].normal, polygon[i].center)
    cstrPolytop = cstr(A,b)
    return cstrPolytop
        

def addRandomPointsCheck(ax, constraintPolygons, nPoints):
    randPoints = 2 * np.random.rand(100,3) - 1.0
    pointsInside = []
    pointsOutside = []
    for i in np.arange(randPoints.shape[0]):
        point = np.array([randPoints[i,0],randPoints[i,1],randPoints[i,2]])
        if checkPointInside(constraintPolygons, randPoints[i,:]):
            pointsInside.append(randPoints[i,:])
        else:
            pointsOutside.append(randPoints[i,:])
    pointsInside = np.array(pointsInside)
    pointsOutside = np.array(pointsOutside)

    #constraint = polygonToMatrixConstraint(constraintPolygons)
    #print('Should be positive')
    #for i in np.arange(pointsInside.shape[0]):
    #    point = pointsInside[i,:]
    #    print(np.all(constraint.A.dot(point.transpose())-constraint.b.transpose() >= 0))
    #print('Should be negative')
    #for i in np.arange(pointsOutside.shape[0]):
    #    point = pointsOutside[i,:]
    #    print(np.all(constraint.A.dot(point.transpose())-constraint.b.transpose() >= 0))

    ax.scatter(pointsInside[:,0],pointsInside[:,1],pointsInside[:,2], c='b', marker='x')
    ax.scatter(pointsOutside[:,0],pointsOutside[:,1],pointsOutside[:,2], c='r', marker='x')
    return ax

#This function should be used only on a complete sphere, not a cut one
def computeNormals(sphereFaces, sphereCenter):
    newFaces = []
    for face in sphereFaces:
        faceCenter = computeFaceCenter(face)
        normal = np.cross(face.vertices[1] - face.vertices[0], face.vertices[2] - face.vertices[1])
        normal = normal/np.linalg.norm(normal)
        if(np.dot(normal, sphereCenter - faceCenter) <= 0):
            normal = -normal
        newFaces.append(flatPolygon(face.vertices, normal, faceCenter))
    return newFaces

def checkPointInside(faces, point):
    for face in faces:
        if np.dot(face.normal, point - face.center)<0:
            return False
    return True

def discretizeSphere(center, radius, nIter):
    pXp = center + radius * np.array([1.0, 0.0, 0.0])
    pXm = center + radius * np.array([-1.0, 0.0, 0.0])
    pYp = center + radius * np.array([0.0, 1.0, 0.0])
    pYm = center + radius * np.array([0.0, -1.0, 0.0])
    pZp = center + radius * np.array([0.0, 0.0, 1.0])
    pZm = center + radius * np.array([0.0, 0.0, -1.0])
    faces = []
    faces.append(flatPolygon([pXp, pYp, pZp],[],[]))
    faces.append(flatPolygon([pXp, pYm, pZp],[],[]))
    faces.append(flatPolygon([pXp, pYp, pZm],[],[]))
    faces.append(flatPolygon([pXp, pYm, pZm],[],[]))
    faces.append(flatPolygon([pXm, pYp, pZp],[],[]))
    faces.append(flatPolygon([pXm, pYm, pZp],[],[]))
    faces.append(flatPolygon([pXm, pYp, pZm],[],[]))
    faces.append(flatPolygon([pXm, pYm, pZm],[],[]))
    iter = 0
    while iter < nIter:
        newFaces = []
        for face in faces:
            m01 = (face.vertices[0] + face.vertices[1])/2.0
            m12 = (face.vertices[1] + face.vertices[2])/2.0
            m20 = (face.vertices[2] + face.vertices[0])/2.0
            m01 = center + radius * (m01-center)/np.linalg.norm(m01-center)
            m12 = center + radius * (m12-center)/np.linalg.norm(m12-center)
            m20 = center + radius * (m20-center)/np.linalg.norm(m20-center)
            newFaces.append(flatPolygon([m01, m20, face.vertices[0]],[],[]))
            newFaces.append(flatPolygon([m12, m01, face.vertices[1]],[],[]))
            newFaces.append(flatPolygon([m20, m12, face.vertices[2]],[],[]))
            newFaces.append(flatPolygon([m01, m12, m20],[],[]))
        faces = newFaces
        iter = iter + 1
    return faces

def computeFaceCenter(face):
    center = np.array([0,0,0])
    for vertex in face.vertices:
        center = center + vertex
    center = center/len(face.vertices)
    return center

def cutSphere(faces, height, sphereCenter):
    newFaces = []
    for face in faces:
        if (face.vertices[0][2]>height and face.vertices[1][2]>height and face.vertices[2][2]>height):
            newFaces.append(face)
        elif (face.vertices[0][2]>height or face.vertices[1][2]>height or face.vertices[2][2]>height):
            cutFace = computeCutFace(face, height)
            cutFaceCenter = computeFaceCenter(cutFace)
            newFaces.append(flatPolygon(cutFace.vertices, face.normal, cutFaceCenter))
    bottomFaceNormal = np.array([0,0,1])
    bottomFaceCenter = np.array([sphereCenter[0], sphereCenter[1], height])
    newFaces.append(flatPolygon([], bottomFaceNormal, bottomFaceCenter))
    return newFaces

def cutSegment(p0, p1, height):
    if(p0[2]>height and p1[2]>height):
        return [p0, p1]
    elif(p0[2]>height or p1[2]>height):
        pUp = []
        pDown = []
        if(p0[2]>height):
            pUp = p0
            pDown = p1
        if(p1[2]>height):
            pUp = p1
            pDown = p0
        alpha = (height - pDown[2])/(pUp[2] - pDown[2])
        pMid = pDown + alpha*(pUp - pDown)
        return [pMid, pUp]

def computeCutFace(face, height):
    newFaceVertices = []
    nPointsAbove = 0
    pointsAbove = []
    pointsBelow = []
    for vertex in face.vertices:
        if(vertex[2]>height):
            nPointsAbove = nPointsAbove + 1
            pointsAbove.append(vertex)
        else:
            pointsBelow.append(vertex)
        
    if(nPointsAbove == 3):
        return face
    elif(nPointsAbove == 1):
        seg0 = cutSegment(pointsAbove[0],pointsBelow[0],height)
        seg1 = cutSegment(pointsAbove[0],pointsBelow[1],height)
        newFaceVertices.append(pointsAbove[0])
        newFaceVertices.append(seg0[0])
        newFaceVertices.append(seg1[0])
    elif(nPointsAbove == 2):
        seg0 = cutSegment(pointsAbove[0],pointsBelow[0],height)
        seg1 = cutSegment(pointsAbove[1],pointsBelow[0],height)
        newFaceVertices.append(pointsAbove[0])
        newFaceVertices.append(pointsAbove[1])
        newFaceVertices.append(seg1[0])
        newFaceVertices.append(seg0[0])
    return flatPolygon(newFaceVertices, face.normal, face.center)

def addFacesToPlot(ax, faces):
    for face in faces:
        if len(face.vertices) > 0:
            lineX = np.array([])
            lineY = np.array([])
            lineZ = np.array([])
            for vertex in face.vertices:
                lineX = np.append(lineX, vertex[0])
                lineY = np.append(lineY, vertex[1])
                lineZ = np.append(lineZ, vertex[2])
            lineX = np.append(lineX, face.vertices[0][0])
            lineY = np.append(lineY, face.vertices[0][1])
            lineZ = np.append(lineZ, face.vertices[0][2])
            ax.plot(lineX, lineY, lineZ)
    return ax

def addNormalsToPlot(ax, faces):
    for face in faces:
        lineX = np.array([face.center[0],face.center[0]+0.1*face.normal[0]])
        lineY = np.array([face.center[1],face.center[1]+0.1*face.normal[1]])
        lineZ = np.array([face.center[2],face.center[2]+0.1*face.normal[2]])
        ax.plot(lineX, lineY, lineZ)
    return ax

def plotFaces(faces):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax = addFacesToPlot(ax, faces)
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.0, 1.0)
    plt.show()

def plotFacesAndNormals(faces):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax = addFacesToPlot(ax, faces)
    ax = addNormalsToPlot(ax, faces)
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.0, 1.0)
    plt.show()




main(args)
