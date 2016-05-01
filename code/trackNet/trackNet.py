#!/usr/bin/python
 
__author__ = ('David Dunn')
__version__ = '0.1'

import cv2
import tensorflow as tf
import trackNet_input as inp

import tensorflow.python.platform
from tensorflow.python.platform import gfile
import logging
import re, os, time, datetime, sys
import numpy as np

# Global flags to control training
DATA_DIR = '/playpen/tracknet'
FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_integer('width', 640, """Width of image input""")
tf.app.flags.DEFINE_integer('height', 480, """Height of image input""")
tf.app.flags.DEFINE_integer('depth', 2, """Number of channels in image input""")
tf.app.flags.DEFINE_float('scale_factor', 1764.7058823529412, """Scale of translations to rotations""")
tf.app.flags.DEFINE_float('keep_prob', 0.5, """Dropout rate for first fully-connected layer""")
tf.app.flags.DEFINE_float('keep_prob2', 1.0, """Dropout rate for second fully-connected layer""")

# Training parameters
tf.app.flags.DEFINE_float('weight_decay', 5e-4, """Weight decay (L2) for the weights""")
tf.app.flags.DEFINE_float('learning_rate', 0.1, """Base Learning rate for all layers""")
tf.app.flags.DEFINE_float('decay_rate', 0.5, """Learning rate decay rate""")
tf.app.flags.DEFINE_integer('decay_step', 27, """How many steps to decay the learning rate""")
tf.app.flags.DEFINE_float('momentum', 0.9, """Momentum used in optimizer""")
tf.app.flags.DEFINE_integer('num_epochs', 1, """Number of Epochs""")
tf.app.flags.DEFINE_integer('batch_size', 50, """Number of pairs to process in a batch.""")
tf.app.flags.DEFINE_integer('iter_print_train_info', 50, """Number of iterations to print training Information  """)
tf.app.flags.DEFINE_string('act','relu', """Activation function used for Student Networks """)

# Helper functions
def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)
def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)
def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')
def conv2d_s2(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 2, 2, 1], padding='SAME')
def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
def max_pool_4x4(x):
    return tf.nn.max_pool(x, ksize=[1, 4, 4, 1], strides=[1, 4, 4, 1], padding='SAME')
def max_pool_5x5(x):
    return  tf.nn.max_pool(x,ksize=[1,5,5,1],strides=[1,5,5,1],padding='SAME')
def max_pool_8x8(x):
    return tf.nn.max_pool(x, ksize=[1, 8, 8, 1], strides=[1, 8, 8, 1], padding='SAME')
def _activation_summary(x):
  """Helper to create summaries for activations.
  Creates a summary that provides a histogram of activations.
  Creates a summary that measure the sparsity of activations.
  Args:
    x: Tensor
  Returns:
    nothing
  """
  # Remove 'tower_[0-9]/' from the name in case this is a multi-GPU training
  # session. This helps the clarity of presentation on tensorboard.
  TOWER_NAME='tower'
  tensor_name = re.sub('%s_[0-9]*/' % TOWER_NAME, '', x.op.name)
  tf.histogram_summary(tensor_name + '/activations', x)
  tf.scalar_summary(tensor_name + '/sparsity', tf.nn.zero_fraction(x))

def model(orig_image, comp_image, keep_prob, keep_prob2):
    print "TrackNet:  building model..."
    # so I think I should not be doing max pool at all in siamese network - do stride instead?
    # 1st siamese convolutional layer - 32 filters
    # in: 2x480x640x2
    # out: 2x240x320x32
    siam_conv1_w = weight_variable([7, 7, 2, 32])
    siam_conv1_b = bias_variable([32])
    orig_conv1 = tf.nn.relu(conv2d_s2(tf.to_float(orig_image), siam_conv1_w) + siam_conv1_b)
    comp_conv1 = tf.nn.relu(conv2d_s2(tf.to_float(comp_image), siam_conv1_w) + siam_conv1_b)
    bias = tf.nn.bias_add(orig_conv1, siam_conv1_b)
    _activation_summary(orig_conv1)
    orig_pool1 = max_pool_2x2(orig_conv1)
    comp_pool1 = max_pool_2x2(comp_conv1)
    '''with tf.variable_scope('conv1') as scope:
        siam_conv1_w = weight_variable([7, 7, 2, 32])
        siam_conv1_b = bias_variable([32])
        orig_conv = conv2d_s2(tf.to_float(orig_image), siam_conv1_w)
        comp_conv = conv2d_s2(tf.to_float(comp_image), siam_conv1_w)
        orig_bias = tf.nn.bias_add(orig_conv, siam_conv1_b)
        comp_bias = tf.nn.bias_add(comp_conv, siam_conv1_b)
        orig_conv1 = tf.nn.relu(orig_bias, name='orig_conv1')
        comp_conv1 = tf.nn.relu(comp_bias, name='comp_conv1')
        _activation_summary(orig_conv1)
    orig_pool1 = max_pool_2x2(orig_conv1)
    comp_pool1 = max_pool_2x2(comp_conv1)
    '''
    # 2nd siamese convolutional layer - 16 filters
    # in: 2x240x320x32 
    # out: 2x240x320x16
    siam_conv2_w = weight_variable([5, 5, 32, 16])
    siam_conv2_b = bias_variable([16])
    orig_conv2 = tf.nn.relu(conv2d(orig_pool1, siam_conv2_w) + siam_conv2_b)
    comp_conv2 = tf.nn.relu(conv2d(comp_pool1, siam_conv2_w) + siam_conv2_b)
    #orig_pool2 = max_pool_2x2(orig_conv2)
    #comp_pool2 = max_pool_2x2(comp_conv2)
    # concatenate left and right together
    # in: 2x240x320x16
    # out: 240x320x32
    concat_layer = tf.concat(3, [orig_conv2, comp_conv2])
    #concat_shape = tf.reshape(concat_layer, [64,40,30])
    # 3rd convolutional layer - 64 filters
    # in: 240x320x32
    # out: 60x80x64
    conv3_w = weight_variable([7, 7, 32, 64])
    conv3_b = bias_variable([64])
    conv3 = tf.nn.relu(conv2d_s2(concat_layer, conv3_w) + conv3_b)
    pool3 = max_pool_2x2(conv3)
    _activation_summary(pool3)
    # 4th convolutional layer - 32 filters
    # in: 60x80x64
    # out: 30x40x32
    conv4_w = weight_variable([5, 5, 64, 32])
    conv4_b = bias_variable([32])
    conv4 = tf.nn.relu(conv2d(pool3, conv4_w) + conv4_b)
    pool4 = max_pool_2x2(conv4)
    # 5th convolutional layer - 16 filters
    # in: 30x40x32
    # out: 30x40x4 = 4800
    conv5_w = weight_variable([5, 5, 32, 4])
    conv5_b = bias_variable([4])
    conv5 = tf.nn.relu(conv2d(pool4, conv5_w) + conv5_b)
    _activation_summary(conv5)
    # flatten
    conv5_flat = tf.reshape(conv5, [-1, 1200])
    # add in some dropout
    conv5_drop = tf.nn.dropout(conv5_flat, keep_prob)
    # 1st fully connected layer
    # in: 4800
    # out: 64
    fc1_w = weight_variable([1200,64])
    fc1_b = bias_variable([64])
    fc1 = tf.nn.relu(tf.matmul(conv5_drop, fc1_w) + fc1_b)
    _activation_summary(fc1)
    # add in some dropout
    fc1_drop = tf.nn.dropout(fc1, keep_prob2)
    # readout layer
    # in: 64
    # out: 6
    fc2_w = weight_variable([64,6])
    fc2_b = bias_variable([6])
    y_conv = tf.matmul(fc1_drop, fc2_w) + fc2_b
    _activation_summary(y_conv)
    return y_conv
    
def lossFunction(logits, labels, scale_factor):
    print "TrackNet:  building loss function..."
    logit_trans, logit_rot = tf.split(1,2,logits)
    label_trans, label_rot = tf.split(1,2,labels)
    trans_loss = tf.nn.l2_loss(tf.sub(logit_trans, label_trans))
    rot_loss = tf.mul(scale_factor, tf.nn.l2_loss(tf.sub(logit_trans, label_trans)))
    return tf.add(trans_loss,rot_loss)
    
def optimizer(loss, learningRate):
    print "TrackNet:  building optimizer..."
    # might switch to momentum optimizer, but just use Adam for now
    optimizer = tf.train.AdamOptimizer(learningRate)
    # Compute the gradients for a list of variables.
    grads_and_vars = optimizer.compute_gradients(loss, var_list=tf.all_variables())
    train_op = optimizer.apply_gradients(grads_and_vars) #, global_step=batch)
    return train_op
    
def train(data_pairs, data_labels, data_images, resume=True):
    dataSize = data_labels.shape[0]
    # setup logging to a file
    logging.basicConfig(filename=os.path.join(DATA_DIR,'logs','train.log'), level=logging.INFO)
    # Write down all the FLAGS
    logging.info('FLAG information')
    for key, value in tf.app.flags.FLAGS.__flags.iteritems():
        logging.info( 'FLAG(%s) : %s'%(key, str(value)))
    logging.info('Training Size = %d', dataSize)
    # setup the model
    orig_image = tf.placeholder(tf.uint8, shape=[None, FLAGS.height, FLAGS.width, FLAGS.depth])
    comp_image = tf.placeholder(tf.uint8, shape=[None, FLAGS.height, FLAGS.width, FLAGS.depth])
    keep_prob = tf.placeholder(tf.float32)
    keep_prob2 = tf.placeholder(tf.float32)
    logits = model(orig_image, comp_image, keep_prob, keep_prob2)
    # setup the loss function
    labels = tf.placeholder(tf.float32, shape=[None, 6])
    scale_factor = tf.placeholder(tf.float32)
    loss = lossFunction(logits, labels, scale_factor)
    tf.add_to_collection('show', loss)
    tf.scalar_summary('loss/total_loss', loss)
    # setup the optimizer
    train_op = optimizer(loss, 1e-4)
    # Build the summary operation based on the TF collection of Summaries.
    summary_op = tf.merge_all_summaries()
    train_loss=[]
    train_error= []
    print "TrackNet:  starting session..."
    with tf.Session() as s:
        # Create a saver to store all the variables later
        saver = tf.train.Saver(tf.all_variables(),keep_checkpoint_every_n_hours=1)
        ckpt = tf.train.get_checkpoint_state(os.path.join(DATA_DIR,'data'))
        if resume and ckpt and ckpt.model_checkpoint_path:
            print "TrackNet:  Loading latest checkpoint %s..."%os.path.join(DATA_DIR,'data',ckpt.model_checkpoint_path)
            saver.restore(s, os.path.join(DATA_DIR,'data',ckpt.model_checkpoint_path))
        else:
            # Run all the initializers to prepare the trainable parameters.
            print "TrackNet:  Initializing all variables..."
            tf.initialize_all_variables().run()
        summary_writer = tf.train.SummaryWriter('logs', graph_def=s.graph)
        logging.info('Initialized!')
        # Loop through training steps.
        numSteps = FLAGS.num_epochs * dataSize // FLAGS.batch_size
        session_start_time = time.time()
        ret = []
        for step in xrange(FLAGS.num_epochs * dataSize // FLAGS.batch_size):
            # Compute the offset of the current minibatch in the data.
            # Note that we could use better randomization across epochs.
            offset = (step * FLAGS.batch_size) % (dataSize - FLAGS.batch_size)
            batch_pairs = data_pairs[offset:(offset + FLAGS.batch_size)] 
            batch_labels = data_labels[offset:(offset + FLAGS.batch_size),:]
            orig_images = data_images[[x for x,y in batch_pairs]]
            comp_images = data_images[[y for x,y in batch_pairs]]
            sys.stdout.write('-')
            sys.stdout.flush()
            # This dictionary maps the batch data (as a np array) to the
            # node in the graph is should be fed to.
            feed_dict = {orig_image: orig_images,
                         comp_image: comp_images,
                         labels: batch_labels,
                         scale_factor: FLAGS.scale_factor,
                         keep_prob: FLAGS.keep_prob,
                         keep_prob2: FLAGS.keep_prob2, 
            }
            start_time = time.time()
            # Run the graph and fetch some of the nodes.
            # Remind : (train_op) is the most parameter here. Without it, Tensorflow will not do the backpropogation.
            ret.append(s.run(tf.get_collection('show')+[logits,train_op],feed_dict=feed_dict))
            duration = time.time() - start_time
            
            if step % FLAGS.iter_print_train_info == (FLAGS.iter_print_train_info-1):
                # Print the training Information
                logging.info('Epoch %.2f, Step %d' % ((float(step) * FLAGS.batch_size / dataSize), step))
                # Print the time information
                sec_per_batch = float(duration)
                remaining_sec  = int(float((time.time() - session_start_time)/step) * float(numSteps - step))
                remaining_time = str(datetime.timedelta(seconds=remaining_sec))
                logging.info('%.3f sec/batch, remaining time = %s' %(sec_per_batch, remaining_time))
                ret = np.array(ret)
                #for idx, var in enumerate(tf.get_collection('show')):
                #    logging.info('Average (%s): %f' % (var.name, np.mean(ret[:, idx])) )
                #logging.info('Average Train error: %.2f%%' % np.mean(train_error))
                #train_error = []
                ret = []
                feed_dict[keep_prob] = 1.0
                feed_dict[keep_prob2] = 1.0
                current_loss = loss.eval(feed_dict=feed_dict)
                #print('\n')
                print("step %d, loss: %s"%(step, current_loss))
                sys.stdout.flush()
                # Save the summary information
                logging.info('Save the summary information')
                summary_str = s.run(summary_op, feed_dict)
                summary_writer.add_summary(summary_str, step)
            if step % 500 == 0:
                # Save the current model
                checkpoint_path = os.path.join(DATA_DIR,'data','model.ckpt')
                logging.info('Save the model : %s'%(checkpoint_path))
                saver.save(s, checkpoint_path, global_step=step)
        # Save the last model
        checkpoint_path = os.path.join(DATA_DIR,'data','model.ckpt')
        logging.info('Save the model : %s'%(checkpoint_path))
        saver.save(s, checkpoint_path, global_step=step)
    
     
def main():
    # load the dataset into memory
    train_pairs, train_labels, train_images = inp.inputs()
    # train the network
    train(train_pairs, train_labels, train_images)
    
if __name__ == '__main__':
    main()