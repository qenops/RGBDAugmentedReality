import time
import datetime
import tensorflow as tf
# datasets
from datasets.cifar10 import *
from datasets.cifar100 import *
from networks.snn30k import *
from networks.vgg16 import *
from networks.snn30k_wo_norm import *


import tensorflow.python.platform
from tensorflow.python.platform import gfile
import logging


FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string('data_dir', './data',
                           """ directory to store all the data and model """)
tf.app.flags.DEFINE_string('dataset', 'cifar10',
                            """ dataset name (cifar10, cifar100) """)
tf.app.flags.DEFINE_string('network', 'snn30k',
                           """ neural network : vgg16, snn30k, snn30_wo_norm""")
tf.app.flags.DEFINE_string('train_model_name', 'temp_sgd',
                           """ directory : ./data/dataset/train_model_name """)

tf.app.flags.DEFINE_integer('d1', 32, """Number of filters for first layer""")
tf.app.flags.DEFINE_integer('d2', 5000, """Number of neurons for the first Fully connected layer""")
tf.app.flags.DEFINE_integer('pair', 2, """Number of pair products""")

# Training parameters
tf.app.flags.DEFINE_float('weight_decay', 5e-4, """Weight decay (L2) for the weights""")
tf.app.flags.DEFINE_float('learning_rate', 0.1, """Base Learning rate for all layers""")
tf.app.flags.DEFINE_float('decay_rate', 0.5, """Learning rate decay rate""")
tf.app.flags.DEFINE_integer('decay_step', 27, """How many steps to decay the learning rate""")
tf.app.flags.DEFINE_float('momentum', 0.9, """Momentum used in optimizer""")
tf.app.flags.DEFINE_integer('num_epochs', 300, """Number of Epochs""")
tf.app.flags.DEFINE_integer('batch_size', 128,
                            """Number of images to process in a batch.""")
tf.app.flags.DEFINE_integer('iter_print_train_info', 50, """Number of iterations to print training Information  """)
tf.app.flags.DEFINE_string('act','relu', """Activation function used for Student Networks """)



TRAIN_MODEL_DIR =os.path.join(FLAGS.data_dir, FLAGS.dataset, FLAGS.train_model_name)


def main(argv=None):

    ## Create the directory for training model
    if gfile.Exists(TRAIN_MODEL_DIR):
        gfile.DeleteRecursively(TRAIN_MODEL_DIR)
    gfile.MakeDirs(TRAIN_MODEL_DIR)

    # Using logging to output and record everything
    # set up logging to file
    util.set_logging(os.path.join(TRAIN_MODEL_DIR, 'myapp.log'))

    # Write down all the FLAGS
    logging.info('FLAG information')
    for key, value in tf.app.flags.FLAGS.__flags.iteritems():
        logging.info( 'FLAG(%s) : %s'%(key, str(value)))


    # Select the dataset
    if FLAGS.dataset == 'cifar10':
        ds = cifar10()
    elif FLAGS.dataset == 'cifar100':
        ds = cifar100()
    else:
        raise ValueError('Wrong dataset name. Check FLAGS.dataset')

    # Download the dataset
    ds.maybe_download()

    # Read data
    train_data, train_labels = ds.read_data(True)

    TRAIN_SIZE = train_labels.shape[0]
    logging.info('Training Size = %d', TRAIN_SIZE)


    # This is where training samples and labels are fed to the graph.
    # These placeholder nodes will be fed a batch of training data at each
    # training step using the {feed_dict} argument to the Run() call below.
    # This part depends on the Dataset
    train_data_node = tf.placeholder(
        tf.float32,
        shape=(FLAGS.batch_size, ds.image_size(), ds.image_size(), ds.num_channel()), name='data_node')
    train_labels_node = tf.placeholder(tf.float32,
                                     shape=(FLAGS.batch_size, ds.num_label()), name='label_node')
    tf.image_summary('images', train_data_node, max_images=FLAGS.batch_size)

    # Training Model Architecture
    # Select the network
    if FLAGS.network == 'vgg16':
        network = vgg16()
    elif FLAGS.network == 'snn30k':
        network = snn30k()
    elif FLAGS.network == 'snn30k_wo_norm':
        network = snn30k_wo_norm()
    else:
        raise ValueError('Wrong dataset name. Check FLAGS.network')




    network_dict= network.model2(data=train_data_node, num_label=ds.num_label() , d1=FLAGS.d1, d2=FLAGS.d2, pair=FLAGS.pair, train=True)
    logits = network_dict['softmax_linear']
    softmax = tf.nn.softmax(logits)
    tf.histogram_summary('logits', logits)
    tf.histogram_summary('softmax',softmax)


    # Define Objective Function
    cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(
        logits, train_labels_node), name='cross_entropy')
    tf.add_to_collection('losses', cross_entropy )
    loss = tf.add_n(tf.get_collection('losses'), name='total_loss')
    tf.add_to_collection('show', cross_entropy)
    tf.add_to_collection('show', loss)
    tf.scalar_summary('loss/total_loss', loss)
    tf.scalar_summary('loss/entropy', cross_entropy)

    # Optimizer: set up a variable that's incremented once per batch and
    # controls the learning rate decay.
    batch = tf.Variable(0)
    # Decay once per epoch, using an exponential schedule starting at 0.01.
    learning_rate = tf.train.exponential_decay(
        FLAGS.learning_rate,                # Base learning rate.
        batch * FLAGS.batch_size ,          # Current index into the dataset.
        TRAIN_SIZE * FLAGS.decay_step,      # Decay step.
        FLAGS.decay_rate,                   # Decay rate.
        staircase=True,
        name='learning_rate'
        )
    #learning_rate = tf.Variable(FLAGS.learning_rate, name='learning_rate')

    tf.scalar_summary("learning_rate", learning_rate)
    tf.add_to_collection('show', learning_rate)
    # Use simple momentum for the optimization.
    optimizer = tf.train.MomentumOptimizer(learning_rate, FLAGS.momentum)
#    optimizer = tf.train.AdamOptimizer(learning_rate)

    # Compute the gradients for a list of variables.
    grads_and_vars = optimizer.compute_gradients(loss, var_list=tf.all_variables())

    # Let Batch normalization variables have higher learning rate
    clipped_grads_and_vars = []
    """
    for gv in grads_and_vars:
        if gv[0] is not None:
            if 'bn_weights' in gv[1].name:
                clipped_grads_and_vars.append([tf.mul(gv[0], tf.constant([1.00001]) ), gv[1]])
            elif 'bn_biases' in gv[1].name:
                clipped_grads_and_vars.append([tf.mul(gv[0], tf.constant([1.00001]) ), gv[1]])
            else:
                clipped_grads_and_vars.append([gv[0] , gv[1]])


    train_op = optimizer.apply_gradients(clipped_grads_and_vars, global_step=batch)
    """
    train_op = optimizer.apply_gradients(grads_and_vars, global_step=batch)

    # Build the summary operation based on the TF collection of Summaries.
    summary_op = tf.merge_all_summaries()

    train_loss=[]
    train_error= []

    # Create a local session to run this computation.
    with tf.Session() as s:
        # Create a saver to store all the variables later
        saver = tf.train.Saver(tf.all_variables())
        # Run all the initializers to prepare the trainable parameters.
        tf.initialize_all_variables().run()

        summary_writer = tf.train.SummaryWriter(TRAIN_MODEL_DIR,
                                            graph_def=s.graph_def)


        offset_old = 0
        logging.info('Initialized!')
        ret = []


        # Loop through training steps.
        num_steps = FLAGS.num_epochs * TRAIN_SIZE // FLAGS.batch_size
        session_start_time = time.time()
        for step in xrange(FLAGS.num_epochs * TRAIN_SIZE // FLAGS.batch_size):
            # Compute the offset of the current minibatch in the data.
            # Note that we could use better randomization across epochs.
            offset = (step * FLAGS.batch_size) % (TRAIN_SIZE - FLAGS.batch_size)
            batch_data = train_data[offset:(offset + FLAGS.batch_size), :, :, :]
            batch_labels = train_labels[offset:(offset + FLAGS.batch_size)]
            # This dictionary maps the batch data (as a np array) to the
            # node in the graph is should be fed to.
            feed_dict = {train_data_node: batch_data,
                   train_labels_node: batch_labels}
            start_time = time.time()
            # Run the graph and fetch some of the nodes.
            # Remind : (train_op) is the most parameter here. Without it, Tensorflow will not do the backpropogation.
            ret.append( s.run(
                tf.get_collection('show')+[logits, train_op],
                feed_dict=feed_dict) )
            duration = time.time() - start_time

            train_error.append( util.error_rate(ret[-1][-2], batch_labels ))

            if step % FLAGS.iter_print_train_info == (FLAGS.iter_print_train_info-1):
                # Print the training Information
                logging.info('Epoch %.2f, Step %d' % ((float(step) * FLAGS.batch_size / TRAIN_SIZE), step))

                # Print the time information
                sec_per_batch = float(duration)
                remaining_sec  = int(float((time.time() - session_start_time)/step) * float(num_steps - step))
                remaining_time = str(datetime.timedelta(seconds=remaining_sec))
                logging.info('%.3f sec/batch, remaining time = %s' %(sec_per_batch, remaining_time))

                ret = np.array(ret)
                for idx, var in enumerate(tf.get_collection('show')):
                    logging.info('Average (%s): %f' % (var.name, np.mean(ret[:, idx])) )

                logging.info('Average Train error: %.2f%%' % np.mean(train_error))

                train_error = []
                ret = []
                print('\n')
                sys.stdout.flush()

            if step % 100==99:
                # Save the summary information
                logging.info('Save the summary information')
                summary_str = s.run(summary_op, feed_dict)
                summary_writer.add_summary(summary_str, step)

            # Per Epoch
            if(offset < offset_old ):
                cur_epoch =np.round((float(step) * FLAGS.batch_size / TRAIN_SIZE))
                cur_epoch = int(cur_epoch)
                logging.info('Epoch %d' % cur_epoch)

                # Randomize Data for Batch normalization
                logging.info('Reorder data order for Batch Normalization')
                rand_idx = np.random.permutation(len(train_labels))
                train_data   = train_data[rand_idx, :, :, :]
                train_labels = train_labels[rand_idx, :]

                # Horizontal mirroring
                logging.info('Randomly horizontal flip the Images')
                mir_idx  = np.random.randint(2, size= len(train_labels))
                mir_idx = np.nonzero(mir_idx > 0 )[0]
                for i in range(ds.num_channel()):
                    train_data[mir_idx, :, :, i ] = train_data[mir_idx, :, ::-1, i]

                if((cur_epoch % 10) == 9):
                    # Save Model
                    checkpoint_path = os.path.join(TRAIN_MODEL_DIR, 'model.ckpt')
                    logging.info('Save the model : %s'%(checkpoint_path))
                    saver.save(s, checkpoint_path, global_step=step)

                sys.stdout.flush()

            offset_old = offset

        # Save the last model
        logging.info('Save the model : %s'%(checkpoint_path))
        saver.save(s, checkpoint_path, global_step=step)


if __name__ == '__main__':
    main()
