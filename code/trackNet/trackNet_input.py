#!/usr/bin/python
 
__author__ = ('David Dunn')
__version__ = '0.1'

import cv2
import tensorflow as tf
import numpy as np
import sys, os, cPickle
sys.path.append(r'..')
import matchmaker
import random

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
USE_COLOR=False

# we need to load the list of tuples - and precompute the labels
# key = pair tuple
# value = 6x1 tensor
def loadDataset(dataDir, pairLabelsFile='pairLabels.txt', preMultFile='hiBall2Cam.txt'):
    if os.path.isfile(os.path.join(dataDir, pairLabelsFile)):
        print "TrackNet:  Loading pairs from %s..."%os.path.join(dataDir, pairLabelsFile)
        size, pairs, labels = cPickle.load(open(os.path.join(dataDir, pairLabelsFile), 'r'))
    else:
        size, pairs, labels = matchmaker.getPairs(dataDir, preMultFile=preMultFile)
        cPickle.dump([size,pairs,labels], open(os.path.join(dataDir, pairLabelsFile), 'w')) 
    # need to combine color channels with depth channel
    print "TrackNet:  Loading images into memory..."
    imageSet = np.empty((size,IMAGE_HEIGHT,IMAGE_WIDTH,2+2*USE_COLOR))
    imgNameFmt = 'img_0_%04d.jpg'
    for i in range(size):
        colorP = os.path.join(dataDir, 'color', imgNameFmt%i)
        depthP = os.path.join(dataDir, 'depth', imgNameFmt%i)
        if os.path.isfile(colorP) and os.path.isfile(depthP):
            color = cv2.imread(colorP,USE_COLOR)
            depth = cv2.imread(depthP,0)
            imageSet[i] = np.dstack((color,depth))
    return pairs, labels, imageSet
    
def loadDatasets(listOfDir):
    allPairs = []
    allLabels = np.empty((0,6))
    allImages = np.empty((0,IMAGE_HEIGHT,IMAGE_WIDTH,2+2*USE_COLOR))
    for idx, dataDir in enumerate(listOfDir):
        pairs, labels, imageSet = loadDataset(dataDir)
        newPairs = [(x+len(allImages),y+len(allImages)) for x, y in pairs]
        allPairs.extend(newPairs)
        allLabels = np.vstack((allLabels, labels))
        allImages = np.vstack((allImages, imageSet))
    return allPairs, allLabels, allImages

def inputs():
    ''' returns a list and two tensors of the form:
    pairs [n tuples of the form (origImageIdx+imageSetOffset, compImageIdx+imageSetOffset)]
    labels [n, 6]
    images [imageID+imageSetOffset, 640, 480, 2]
    '''
    # load datasets into memory 
    listOfDir = [
        '/playpen/tracknet/radialCircularWalk/',
    ]
    pairs, labels, images = loadDatasets(listOfDir)
    # should we shuffle pairs and labels first?
    
    return pairs, labels, images