import cv2
import numpy as np
from jetracer.nvidia_racecar import NvidiaRacecar

car = NvidiaRacecar()

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

# Steering publish function (actual implementation may vary depending on the environment)
def publish_steering(value):
    print(f"Publishing steering: {value}")
    # Implement actual publishing logic here

if cap.isOpened():
    while True:
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
        wvX = np.arange(0, w)
        wvXX = wvX - (w / 2)
        wvY = np.arange(0,h).reshape(h,1)
        tresh = result[:,:,0] / 255.0  # Use single channel
        
        tresh12 = tresh * wvXX * wvY
        lineDetect = np.sum(tresh12)
        
        # Calculate the slicing indices
        height_start = h - (2 * h // 3)
        height_end = h
        width_start = 0
        width_end = w // 3
        
        # Extract the bottom-left portion
        bottom_left = mask[height_start:height_end, width_start:width_end]
        bottom_left_sum = np.sum(bottom_left)
        
        # Calculate the slicing indices
        height_start_1 = (1 * h // 3)
        height_end_1 =  h - (1 * h // 3)  # This line was incorrect
        width_start_1 = 0
        width_end_1 = w // 9
        
        # Extract the bottom-left portion
        bottom_left_1 = mask[height_start_1:height_end_1, width_start_1:width_end_1]
        bottom_left_sum_1 = np.sum(bottom_left_1)
        
        print(bottom_left_sum)
        #car.throttle = 0.2
        


        if bottom_left_sum < 1000000:
            print("left")
            car.steering = -0.65
        else:
            print("gogogo")
            car.steering = -0.4

        
        cv2.imshow('q1', bottom_left)
        cv2.imshow('q11', bottom_left_1)
        
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            car.throttle = 0
            break
else:
    print("Can't open camera.")

cap.release()
cv2.destroyAllWindows()
