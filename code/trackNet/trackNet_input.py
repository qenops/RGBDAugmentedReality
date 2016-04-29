#!/usr/bin/python
 
__author__ = ('David Dunn')
__version__ = '0.1'

import cv2
import tensorflow as tf
import sys
sys.path.append(r'..')
import matchmaker

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# we need to load the list of tuples - and precompute the labels
# key = pair tuple
# value = 6x1 tensor

def read_trackNet(filename_queue):
    pass
    
def inputs():
    # load datasets into memory
    pairLabels, imageSet = matchmaker.loadDataset('/playpen/tracknet/radialCircularWalk/')
    # create a queue?
    
    # read from files in queue
    #read_input = read_trackNet(filename_queue)