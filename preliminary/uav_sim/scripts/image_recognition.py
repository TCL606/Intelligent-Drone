import cv2
import numpy as np
def image_recognition(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #Red color range 0, 235, 120, 10, 250, 130
    red_lower_range = np.array([0, 235, 120])
    red_upper_range = np.array([10, 250, 130])
    #Blue color range  110, 50, 50 , 130, 255, 255
    blue_lower_range = np.array([110, 50, 50])
    blue_upper_range = np.array([130, 255, 255])
    #Yellow color range  20, 43, 46 , 34, 255, 255
    yellow_lower_range = np.array([20, 43, 46])
    yellow_upper_range = np.array([34, 255, 255])

    red_mask = cv2.inRange(hsv, red_lower_range, red_upper_range)
    blue_mask = cv2.inRange(hsv, blue_lower_range, blue_upper_range)
    yellow_mask = cv2.inRange(hsv, yellow_lower_range, yellow_upper_range)
#cv2.imwrite("./red_mask.png", red_mask)

    if (np.sum(blue_mask==255)>2000):
        return 'b' # blue
    elif (np.sum(yellow_mask==255)>2000):
        return 'y' # yellow
    elif (np.sum(img[:,:,0] + img[:,:,1]== 0) + np.sum(red_mask==255)>2000):
        return 'r' # red
    else:
        return 'e' # error