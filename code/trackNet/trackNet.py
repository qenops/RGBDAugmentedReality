#!/usr/bin/python
 
__author__ = ('David Dunn')
__version__ = '0.1'

import tensorflow as tf


image_left = tf.placeholder(tf.float32, shape=[None, 64, 64, 1])
image_right = tf.placeholder(tf.float32, shape=[None, 64, 64, 1])

pool_left = tf.nn.max_pool(image_left, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
pool_right = tf.nn.max_pool(image_left, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

pool_flat_left = tf.reshape(pool_left, [-1, 32*32])
pool_flat_right = tf.reshape(pool_right, [-1, 32*32])

concat_layer = tf.concat(1, [pool_flat_left, pool_flat_right])