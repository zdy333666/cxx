import cv2
import time
import numpy as np
import tensorflow as tf

from skimage.transform import resize
from keras.models import load_model

import detect_face

import gc


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
                self.pnet, self.rnet, self.onet = detect_face.create_mtcnn(sess, "./data/")

    def detect_face(self, image):
        bounding_boxes, points = detect_face.detect_face(image, self.minsize, self.pnet, self.rnet, self.onet,
                                                         self.threshold, self.factor)

        return bounding_boxes, points


# 创建MTCNN网络
face_detect = Facedetection()  # 初始化 Facedetection()

# 初始化facenetEmbedding
model_path = './data/facenet_keras.h5'
facenet = load_model(model_path)
# facenet.summary()


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


# obtaining frames from camera--->converting to gray--->converting to rgb
# --->detecting faces---->croping faces--->embedding--->classifying--->print

nowTime = lambda: int(round(time.time() * 1000))

frame_interval = 3
margin = 10
image_size = 160

video_capture = cv2.VideoCapture(0)
c = 0

while True:
    # Capture frame-by-frame

    ret, frame = video_capture.read()
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # print(frame.shape)

    timeF = frame_interval

    if (c % timeF == 0):  # frame_interval==3, face detection every 3 frames
        find_results = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if gray.ndim == 2:
            img = to_rgb(gray)

            # print("img.shape:", img.shape)

            start_time = nowTime()

            bounding_boxes, points = face_detect.detect_face(img)

            end_time = nowTime()
            print("face_detect in ", end_time - start_time, " ms")

            nrof_faces = bounding_boxes.shape[0]  # number of faces

            if nrof_faces > 0:
                print('找到人脸数目为：{}'.format(nrof_faces))

                faces = []

                for face_position in bounding_boxes:
                    face_position = face_position.astype(int)
                    print("face_position:", face_position)
                    # print("face_position min:", np.min(face_position))

                    # Draw a rectangle around the faces
                    cv2.rectangle(frame, (face_position[0], face_position[1]), (face_position[2], face_position[3]),
                                  (0, 255, 0), 2)

                    left = max(face_position[0], 0)
                    right = max(face_position[2], 0)
                    top = max(face_position[1], 0)
                    bottom = max(face_position[3], 0)

                    crop = resize(img[top:bottom, left:right, :], (160, 160), mode='reflect')

                    # crop = img[face_position[1]:face_position[3], face_position[0]:face_position[2], ]
                    # crop = cv2.resize(crop, (image_size, image_size), interpolation=cv2.INTER_CUBIC)
                    # data = crop.reshape(-1, image_size, image_size, 3)

                    # print("crop:", crop.shape)

                    faces.append(crop)

                start_time = nowTime()

                embeddings = calc_embs(faces, len(faces))

                end_time = nowTime()
                print("calc_embs in ", end_time - start_time, " ms")

                print("embeddings.shape:", embeddings.shape)
                # for i in range(len(embeddings)):
                #     print(embeddings[i])

    c += 1

    # Display the resulting frame
    cv2.imshow('Video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    gc.collect()

# When everything is done, release the capture

video_capture.release()
cv2.destroyAllWindows()
