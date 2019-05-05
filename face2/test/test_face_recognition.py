import tensorflow as tf

import detect_face
import facenet

import cv2
import time



class Facedetection:
    def __init__(self):
        self.minsize = 20
        self.threshold = [0.6, 0.7, 0.7]
        self.factor = 0.709

        with tf.Graph().as_default():
            sess = tf.Session()
            with sess.as_default():
                self.pnet, self.rnet, self.onet = detect_face.create_mtcnn(sess, "../data/")

    def detect_face(self, image):
        bounding_boxes, points = detect_face.detect_face(image, self.minsize, self.pnet, self.rnet, self.onet,
                                                         self.threshold, self.factor)

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
20180408-102900
20180402-114759
model_path = "../data/20180402-114759"
face_net = facenetEmbedding(model_path)

# obtaining frames from camera--->converting to gray--->converting to rgb
# --->detecting faces---->croping faces--->embedding--->classifying--->print

nowTime = lambda: int(round(time.time() * 1000))


frame_interval = 3
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
            img = facenet.to_rgb(gray)

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


                    cv2.rectangle(frame, (face_position[0], face_position[1]),
                                  (face_position[2], face_position[3]), (0, 255, 0), 2)

                    left = max(face_position[0], 0)
                    right = max(face_position[2], 0)
                    top = max(face_position[1], 0)
                    bottom = max(face_position[3], 0)

                    # crop = resize(img[top:bottom, left:right, :], (160, 160), mode='reflect')

                    crop = img[top:bottom, left:right, ]
                    crop = cv2.resize(crop, (image_size, image_size), interpolation=cv2.INTER_CUBIC)
                    data = crop.reshape(image_size, image_size, 3)

                    faces.append(data)

                start_time = nowTime()

                embedding = face_net.get_embedding(faces)

                end_time = nowTime()
                print("calc_embs in ", end_time - start_time, " ms")

                print("embedding.shape:", embedding.shape)



    # print(faces)
    c += 1
    # Draw a rectangle around the faces

    # Display the resulting frame

    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture

video_capture.release()
cv2.destroyAllWindows()