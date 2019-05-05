import cv2

import numpy as np
import time
import datetime


def to_rgb(img):
    w, h = img.shape
    ret = np.empty((w, h, 3), dtype=np.uint8)
    ret[:, :, 0] = ret[:, :, 1] = ret[:, :, 2] = img
    return ret


image_path = "../tmp/092234176096410.jpg"

img = cv2.imread(image_path, cv2.IMREAD_COLOR)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = to_rgb(gray)

cv2.namedWindow("image")
cv2.imshow('image', gray)

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# t = time.time()

# nowTime = lambda: int(round(time.time() * 1000))
#
# now = nowTime()
#
# time.sleep(0.01)
#
# now2 = nowTime()
#
# print("now-now2:", now2 - now)
