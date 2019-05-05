from tensorflow.python.client import device_lib
print(device_lib.list_local_devices())



import tensorflow as tf

# with tf.device('/gpu:0'):
a = tf.constant([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], shape=[2, 3], name='a')
b = tf.constant([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], shape=[3, 2], name='b')
c = tf.matmul(a, b)
sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=False, log_device_placement=True))
print(sess.run(c))


# import os
# from tensorflow.python.client import device_lib
# os.environ["TF_CPP_MIN_LOG_LEVEL"] = "99"
#
# if __name__ == "__main__":
#     print(device_lib.list_local_devices())
