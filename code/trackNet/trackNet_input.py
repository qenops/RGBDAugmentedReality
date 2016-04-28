#!/usr/bin/python
 
__author__ = ('David Dunn')
__version__ = '0.1'

import tensorflow as tf
import cv2

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

class depthImage(object):
    width = IMAGE_WIDTH
    height = IMAGE_HEIGHT
    depth = 4               # 4 channels for a depth image
    label_bytes = 48        # 24 for translation 24 for rotation
    @property
    def nbytes(self):
        return (self.width * self.height * self.depth) + self.label_bytes

class trackNetRecord(object):
    origFrame = depthImage()
    newFrame = depthImage()
    @property
    def nbytes(self):
        return self.origFrame.nbytes + self.newFrame.nbytes

def read_trackNet(filename_queue):
    
    
    reader = tf.FixedLengthRecordReader(record_bytes=record_bytes)
    result.key, value = reader.read(filename_queue)
    
def inputs():
    # create a queue?
    
    # read from files in queue
    read_input = read_trackNet(filename_queue)