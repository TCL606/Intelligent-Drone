import cv2
import numpy as np
import math
import time


def seg_circle(img):
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    h, w = img.shape[:2]
    # result = np.zeros([h, w, 3])
    result = img
    circles = detect_circle(img)
    # print(circles)
    print("center:", circles[0][0], circles[0][1])
    print("radius:", circles[0][2])
    for i in range(h):
        for j in range(w):
            if not distance(i, j, circles[0][1],
                            circles[0][0]) < circles[0][2]:
                result[i][j][:] = [0, 0, 0]
                # result[i][j][:] = img[i][j][:]
            # else:
    result.astype(np.uint8)
    cv2.imwrite('result.jpg', result)
    return result, circles[0][2]


def detect_circle(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gaussian = cv2.GaussianBlur(gray, (3, 3), 0)
    circles1 = cv2.HoughCircles(gaussian,
                                cv2.HOUGH_GRADIENT,
                                1,
                                100,
                                param1=200,
                                param2=30,
                                minRadius=0,
                                maxRadius=0)
    circles = circles1[0, :, :]
    circles = np.uint16(np.around(circles))
    return circles


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def ball_detect(img):
    result, radius = seg_circle(img)
    if radius > 150:
        return -1


def image_recognition(img, radius):
    height = img.shape[0]
    width = img.shape[1]

    # Red color range 0, 235, 120, 10, 250, 130
    # red_range = [(156, 43, 46), (180, 255, 255)]
    red_range = [(0, 43, 46), (10, 255, 255)]
    blue_range = [(94, 80, 2), (120, 255, 255)]
    yellow_range = [(20, 43, 46), (34, 255, 255)]

    frame = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC) 
    frame = cv2.GaussianBlur(frame, (3, 3), 0)  
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
    h, s, v = cv2.split(frame) 
    v = cv2.equalizeHist(v) 
    frame = cv2.merge((h, s, v))  

    red_frame = cv2.inRange(frame, red_range[0], red_range[1])  
    blue_frame = cv2.inRange(frame, blue_range[0],
                             blue_range[1]) 
    yellow_frame = cv2.inRange(frame, yellow_range[0],
                               yellow_range[1])  

    red_opened = cv2.morphologyEx(red_frame, cv2.MORPH_OPEN,
                                  np.ones((3, 3), np.uint8))
    red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE,
                                  np.ones((3, 3), np.uint8)) 
    red_closed[((img[:, :, 0] < 8) & (img[:, :, 1] < 8) &
                (img[:, :, 2] > 100))] = 255  # modified red_closed with RGB
    (red_contours, _) = cv2.findContours(red_closed, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_NONE) 

    blue_opened = cv2.morphologyEx(blue_frame, cv2.MORPH_OPEN,
                                   np.ones((3, 3), np.uint8))  
    blue_closed = cv2.morphologyEx(blue_opened, cv2.MORPH_CLOSE,
                                   np.ones((3, 3), np.uint8))  
    (blue_contours, _) = cv2.findContours(blue_closed, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_NONE) 

    yellow_opened = cv2.morphologyEx(yellow_frame, cv2.MORPH_OPEN,
                                     np.ones((3, 3), np.uint8))  
    yellow_closed = cv2.morphologyEx(yellow_opened, cv2.MORPH_CLOSE,
                                     np.ones((3, 3), np.uint8))  
    (yellow_contours, _) = cv2.findContours(yellow_closed, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_NONE) 
    # cv2.imwrite("./red_new.jpg", red_closed)
    # cv2.imwrite("./yellow_new.jpg", yellow_closed)
    # cv2.imwrite("./blue_new.jpg", blue_closed)

    if np.sum(red_closed == 255) > math.pi * radius**2 / 2:
        return 0  # b
    elif np.sum(yellow_closed == 255) + np.sum(
            blue_closed == 255) > math.pi * radius**2 / 5:
        return 2  # v
    else:
        return 1  # f


def detect_ball_huff(img):
    result, radius = seg_circle(img)
    if radius > int(max(img.shape[0], img.shape[1]) / 6) or radius < int(
            max(img.shape[0], img.shape[1]) / 50):
        return -1
    else:
        return image_recognition(result, radius)
