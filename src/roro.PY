import cv2
import numpy as np
import time
from jetracer.nvidia_racecar import NvidiaRacecar
car = NvidiaRacecar()
throt = 0.1
qaws = 0
def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('HSV Trackbars')
cv2.createTrackbar('H_min', 'HSV Trackbars', 0, 179, nothing)
cv2.createTrackbar('S_min', 'HSV Trackbars', 0, 255, nothing)
cv2.createTrackbar('V_min', 'HSV Trackbars', 250, 255, nothing)
cv2.createTrackbar('H_max', 'HSV Trackbars', 179, 179, nothing)
cv2.createTrackbar('S_max', 'HSV Trackbars', 15, 255, nothing)
cv2.createTrackbar('V_max', 'HSV Trackbars', 255, 255, nothing)

asdf = 0.19
flag = 0
if cap.isOpened():
    while True:
        print(qaws)
        print(flag)
        qaws = qaws + 1
        if qaws > 25:
            if flag == 0:
                car.throttle = 0.0
                time.sleep(3)
                flag = 1
        if qaws > 50:
            if flag == 1:
                print("qqqqqqqqqqqqqqqqqqq")
                car.throttle = 0.0
                car.throttle = 0.0
                time.sleep(5)
                flag = 2
        if qaws > 70:
            if flag == 2:
                print("qqqqqqqqqqqqqqqqqqq")
                car.throttle = 0.23
                car.steering = -1.0
                flag = 4
        ret, frame = cap.read()
        if not ret:
            print('No frame')
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        h_min = cv2.getTrackbarPos('H_min', 'HSV Trackbars')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Trackbars')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Trackbars')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Trackbars')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Trackbars')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Trackbars')
        
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        h, w = mask.shape
        
        # Calculate the slicing indices
        height_start = h - (2 * h // 3)
        height_end = h
        width_start = 0
        width_end = w // 3
        
        # Extract the bottom-left portion
        bottom_left = mask[height_start:height_end, width_start:width_end]
        bottom_left_sum = np.sum(bottom_left)
        
        # Calculate the slicing indices for the second region
        height_start_1 = (1 * h // 3)
        height_end_1 = h - (1 * h // 3)
        width_start_1 = 0
        width_end_1 = w // 9
        
        # Extract the second bottom-left portion
        bottom_left_1 = mask[height_start_1:height_end_1, width_start_1:width_end_1]
        bottom_left_sum_1 = np.sum(bottom_left_1)
        if throt < asdf:
            throt = throt + 0.05
        else:
            throt = asdf
        print(throt)
        car.throttle = throt
        # t0.23 r0.4 l-0.75 g0.41
        # 0.21  0.4  -0.75  0.41
        if bottom_left_sum > 20000000 and bottom_left_sum_1 < 5000:
            print("right")
            car.steering = 0.4
        else:
            if bottom_left_sum < 20000000:
                print("left")
                car.steering = -0.7
            else:
                print("gogogo")
                car.steering = -0.36
        
        #cv2.imshow('q1', bottom_left)
        #cv2.imshow('q12', frame)
        #cv2.imshow('q11', bottom_left_1)
        
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            car.throttle = 0
            break
else:
    print("Can't open camera.")

cap.release()
cv2.destroyAllWindows()
