import os
import cv2
import time
import numpy as np
import tensorflow as tf

from bson.objectid import ObjectId
from skimage.transform import resize
from keras.models import load_model

import detect_face
import facenet


def to_rgb(img):
    w, h = img.shape
    ret = np.empty((w, h, 3), dtype=np.uint8)
    ret[:, :, 0] = ret[:, :, 1] = ret[:, :, 2] = img
    return ret


class Facedetection:
    def __init__(self):
        self.minsize = 20
        self.threshold = [0.6, 0.7, 0.7]
        self.factor = 0.709

        # self.minsize = 40  # minimum size of face
        # self.threshold = [0.6, 0.7, 0.9]  # three steps's threshold
        # self.factor = 0.709  # scale factor

        with tf.Graph().as_default():
            sess = tf.Session()
            with sess.as_default():
                self.pnet, self.rnet, self.onet = detect_face.create_mtcnn(sess, "data/")

    def detect_face(self, image):
        bounding_boxes, points = detect_face.detect_face(image, self.minsize, self.pnet, self.rnet, self.onet, self.threshold, self.factor)
        return bounding_boxes, points


class facenetEmbedding:
    def __init__(self, model_path):
        self.sess = tf.InteractiveSession()
        self.sess.run(tf.global_variables_initializer())

        facenet.load_model(model_path)

        self.images_placeholder = tf.get_default_graph().get_tensor_by_name("input:0")
        self.tf_embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")
        self.phase_train_placeholder = tf.get_default_graph().get_tensor_by_name("phase_train:0")

    def get_embedding(self, images):
        feed_dict = {self.images_placeholder: images, self.phase_train_placeholder: False}
        embedding = self.sess.run(self.tf_embeddings, feed_dict=feed_dict)
        return embedding

    def free(self):
        self.sess.close()



# 创建MTCNN网络
face_detect = Facedetection()  # 初始化 Facedetection()

# 初始化facenetEmbedding
# model_path = 'data/facenet_keras.h5'
# facenet = load_model(model_path)
# # facenet.summary()

model_path = "data/20180402-114759"
face_net = facenetEmbedding(model_path)


# 初始化加载模型时,预加载 graph
# facenet.predict_on_batch(np.array([np.zeros((160, 160, 3))]))


def prewhiten(x):
    if x.ndim == 4:
        axis = (1, 2, 3)
        size = x[0].size
    elif x.ndim == 3:
        axis = (0, 1, 2)
        size = x.size
    else:
        raise ValueError('Dimension should be 3 or 4')

    mean = np.mean(x, axis=axis, keepdims=True)
    std = np.std(x, axis=axis, keepdims=True)
    std_adj = np.maximum(std, 1.0 / np.sqrt(size))
    y = (x - mean) / std_adj
    return y


def l2_normalize(x, axis=-1, epsilon=1e-10):
    output = x / np.sqrt(np.maximum(np.sum(np.square(x), axis=axis, keepdims=True), epsilon))
    return output


def calc_embs(imgs, batch_size):
    aligned_images = prewhiten(np.array(imgs))

    pd = []
    for start in range(0, len(aligned_images), batch_size):
        pd.append(facenet.predict_on_batch(aligned_images[start:start + batch_size]))

    embs = l2_normalize(np.concatenate(pd))
    return embs


nowTime = lambda: int(round(time.time() * 1000))

image_size = 160


def detect(image, max_face_num):

    tmp_image_path = "tmp/" + ObjectId().__str__() + ".jpg"
    # print("tmp_image_name:", tmp_image_path)

    start_time = nowTime()

    tmp_image_file = open(tmp_image_path, mode='bw+')
    tmp_image_file.write(image)
    tmp_image_file.flush()
    tmp_image_file.close()

    loaded_img = cv2.imread(tmp_image_path, cv2.IMREAD_REDUCED_COLOR_2)
    os.remove(tmp_image_path)

    end_time = nowTime()
    print("save/reload image in ", end_time - start_time, " ms")

    gray = cv2.cvtColor(loaded_img, cv2.COLOR_BGR2GRAY)
    img = to_rgb(gray)
    # img = cv2.cvtColor(loaded_img, cv2.COLOR_BGR2RGB)

    start_time = nowTime()

    bounding_boxes, points = face_detect.detect_face(img)

    end_time = nowTime()
    print("face_detect in ", end_time - start_time, " ms")

    nrof_faces = bounding_boxes.shape[0]  # number of faces

    if nrof_faces > 0:
        print('找到人脸数目为：{}'.format(nrof_faces))

        # print("points ", points)

        faces = []

        for face_position in bounding_boxes:
            face_position = face_position.astype(int)
            print("face_position:", face_position)
            # print("face_position min:", np.min(face_position))

            left = max(face_position[0], 0)
            right = max(face_position[2], 0)
            top = max(face_position[1], 0)
            bottom = max(face_position[3], 0)

            crop = resize(img[top:bottom, left:right, :], (160, 160), mode='reflect')

            faces.append(crop)

        start_time = nowTime()

        # embeddings = calc_embs(faces, len(faces))
        embeddings = face_net.get_embedding(faces)

        end_time = nowTime()
        print("calc_embs in ", end_time - start_time, " ms")

        print("embeddings.shape:", embeddings.shape)


        # for i in range(len(embeddings)):
        #     print(embeddings[i])

    return 0
