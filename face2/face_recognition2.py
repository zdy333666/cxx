import os
import cv2
import time
import numpy as np
import tensorflow as tf

from bson.objectid import ObjectId
from skimage.transform import resize
from annoy import AnnoyIndex

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


def nowTime():
    return int(round(time.time() * 1000))


image_size = 160

# 创建MTCNN网络
face_detect = Facedetection()

# 初始化facenetEmbedding
model_path = "data/20180402-114759"
face_net = facenetEmbedding(model_path)


"""
group_id link to  index_user_map
"""
group_user_map = dict()

"""
group_id link to face feature index
"""
group_index_map = dict()


def build_group_index(group_id, user_ids, feature_list):
    """
    build group feature index
    """

    print("build face index of group_id:", group_id)

    f = 512
    index = AnnoyIndex(f)

    for n in range(len(feature_list)):
        features = feature_list[n]
        index.add_item(n, features)

    index.build(4)

    old_index = group_index_map[group_id]

    if old_index is not None :
        print("old_index:" , old_index)

    group_index_map[group_id] = index

    index_user_map = dict()

    for m in range(len(user_ids)) :
        index_user_map[m] = user_ids[m]

    old_index_user_map = group_user_map[group_id]

    if old_index_user_map is not None:
        print("old_index_user_map:", old_index_user_map)

    group_user_map[group_id] = index_user_map


def local_load_image(image, log_id):

    tmp_image_path = "tmp/" + log_id + ".jpg"
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

    return loaded_img


def local_get_faces(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img = to_rgb(gray)
    # img = cv2.cvtColor(loaded_img, cv2.COLOR_BGR2RGB)

    start_time = nowTime()

    bounding_boxes, points = face_detect.detect_face(img)

    end_time = nowTime()
    print("face_detect in ", end_time - start_time, " ms")

    return bounding_boxes, points


def local_get_descriptors(faces):

    start_time = nowTime()

    embeddings = face_net.get_embedding(faces)

    end_time = nowTime()
    print("calc_embs in ", end_time - start_time, " ms")

    print("embeddings.shape:", embeddings.shape)

    return embeddings


def local_ann_search(descriptors,  result_n, group_id):

    closest = list()

    # group_id link to face feature index
    index = group_index_map[group_id]
    if index is None:
        return closest

    start_time = nowTime()

    # search face from annoy index tree
    search_k = -1
    tmp_closest = list()
    distances = list()

    index.get_nns_by_vector(descriptors, result_n, search_k, tmp_closest, distances)

    end_time = nowTime()
    print("face search with group_id:" , group_id, " in ", end_time - start_time, " ms")

    for i in range(len(tmp_closest)):
        _id = tmp_closest[i]
        distance = distances[i]

        # set the max distance
        if distance > 0.2:
            continue

        closest.append(_id)

        print("search result -- id:", _id, "  distance:", distance)

    return closest


def user_search(descriptors, result_n, group_ids):

    user_infos = list()

    closest = list()
    for group_id in group_ids:

        local_ann_search(closest, descriptors, result_n, group_id)

        for id in closest :
            user_info = dict()
            user_info['group_id'] = group_id
            user_info['user_id'] = (group_user_map[group_id])[id]
            user_info['score'] = 0.0

            user_infos.append(user_info)

        result_n = result_n - len(closest)
        closest.clear()

        if result_n == 0:
            break

    return user_infos


def detect(image, max_face_num):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img = to_rgb(gray)
    # img = cv2.cvtColor(loaded_img, cv2.COLOR_BGR2RGB)

    bounding_boxes, points = local_get_faces(img)

    nrof_faces = bounding_boxes.shape[0]  # number of faces

    if nrof_faces > 0:
        print('找到人脸数目为：{}'.format(nrof_faces))

        faces = []

        for face_position in bounding_boxes:

            face_position = face_position.astype(int)
            print("face_position:", face_position)

            left = max(face_position[0], 0)
            right = max(face_position[2], 0)
            top = max(face_position[1], 0)
            bottom = max(face_position[3], 0)

            crop = resize(img[top:bottom, left:right, :], (160, 160), mode='reflect')

            faces.append(crop)

        embeddings = local_get_descriptors(faces)

        face_infos = list()

        for i in range(len(embeddings)):

            if i > max_face_num:
                break

            face_position = bounding_boxes[i]
            face_position = face_position.astype(int)

            left = max(face_position[0], 0)
            right = max(face_position[2], 0)
            top = max(face_position[1], 0)
            bottom = max(face_position[3], 0)

            location = dict()
            location['left'] = float(left)
            location['top'] = float(top)
            location['width'] = float(right-left)
            location['height'] = float(bottom-top)
            location['rotation'] = 0

            landmarks = list()

            embedding = embeddings[i]
            embedding = embedding.astype(float)

            # print("embedding type:", type(embedding))

            descriptors = list()
            for n in embedding:
                descriptors.append(n)

            face_info = dict()
            face_info['face_token'] = ObjectId().__str__()
            face_info['face_probability'] = 1.0
            face_info['label'] = ''
            face_info['location'] = location
            face_info['landmark'] = landmarks
            face_info['descriptors'] = descriptors

            face_infos.append(face_info)

    return face_infos

