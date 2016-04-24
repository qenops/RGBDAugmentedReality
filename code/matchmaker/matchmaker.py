#!/usr/bin/python
 
__author__ = ('David Dunn')
__version__ = '0.1'

import numpy as np
import cv2
from scipy.spatial.distance import pdist, squareform
import math
from os import path

dataDir = path.expanduser('/workspace/data/radialCircularWalk/')
maxTrans = 1500 # units in mm
maxRot = .85 # units in radians

# parse the tracking data into memory
def parseTrackingFile(path=dataDir, fileName='tracking.txt', preMult=np.matrix(np.eye(4)), postMult=np.matrix(np.eye(4))):
    with open('%s%s'%(path,fileName)) as f:
        numFrames = int(f.readline())
        rotation = np.zeros((numFrames+1, 9),dtype=float)
        rotation[:,0] = 1  # default to identity
        rotation[:,4] = 1
        rotation[:,8] = 1
        #rotation = np.repeat(np.eye(3).reshape((1,3,3)),size,axis=0)
        translation = np.zeros((numFrames+1, 3),dtype=float)
        for line in f:
            tok = line.split(',')
            matrix = np.matrix([float(s) for s in tok[1:]]).reshape((4,4))
            out = preMult*matrix*postMult
            rotation[int(tok[0])] = out.A[:3,:3].reshape((1,9))
            #rotation[int(tok[0])] = out.A[:3,:3]
            translation[int(tok[0])] = out.A[3,:3]
    return rotation, translation
    
# take in two rotation matricies (as arrays) and find the rotation between them    
def distanceRotation(a1, a2):
    r1 = np.matrix(a1.reshape((3,3))).T
    r2 = np.matrix(a2.reshape((3,3))).T
    # I need to do math here and just return a scalar
    rod = cv2.Rodrigues(r1.I*r2)[0]
    #return abs(rod[0])+abs(rod[1])
    return rod

def distances(rotation, translation):
    # just use euclidean distance for translation
    distTrans = pdist(translation, 'euclidean')
    
    distRot = pdist(rotation, distanceRotation)
    return squareform(distRot), squareform(distTrans)

def parseMatrixFile(path=dataDir, fileName='matrix.txt'):
    with open('%s%s'%(path,fileName)) as f:
        for line in f:
            tok = line.split(',')
            matrix = np.matrix([float(s) for s in tok]).reshape((4,4))
    return matrix
    
def getPairs(poseFile='poses.txt',preMultFile=None,postMultFile=None):
    postMult = parseMatrixFile(fileName=postMultFile) if postMultFile is not None else np.eye(4)
    preMult = parseMatrixFile(fileName=preMultFile) if preMultFile is not None else np.eye(4)
    rotation, translation = parseTrackingFile(fileName='poses.txt',preMult=preMult,postMult=postMult)
    distRot, distTrans = distances(rotation, translation)
    notIdentity = np.repeat(translation.all(axis=1),size).reshape((size,size))
    idx = np.flatnonzero(np.all((distRot < maxRot, distTrans < maxTrans, notIdentity),axis=0))
    pairs = np.hstack((idx.reshape((len(idx),1))/size,idx.reshape((len(idx),1))%size))
    return pairs