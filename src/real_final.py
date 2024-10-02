
import time
import serial
import cv2
import numpy as np
from jetracer.nvidia_racecar import NvidiaRacecar
ret = None
cap = None
frame = None
car = NvidiaRacecar()
throt = None
bottom_left_sum = None
bottom_left_sum_1 = None
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
def line_stop():
    car.throttle = 0.0
    print("line_stop")
    
# Steering publish function (actual implementation may vary depending on the environment)
def publish_steering(value):
    print(f"Publishing steering: {value}")
    # Implement actual publishing logic here

def drive_shit():
    global throt 
    global bottom_left_sum
    global bottom_left_sum_1
    if throt < 0.22:
        throt = throt+0.001
    else:
        throt = 0.22
    print(bottom_left_sum)
    car.throttle = 0.0
    # t0.23 r0.4 l-0.75 g0.41
    # 0.21  0.4  -0.75  0.41
    if bottom_left_sum > 20000000 and bottom_left_sum_1 < 5000:
        print("right")
        car.steering = 0.4
    else:
        if bottom_left_sum < 20000000:
            print("left")
            car.steering = -0.75
        else:
            print("gogogo")
            car.steering = -0.41
        
    
    #cv2.imshow('q1', bottom_left)
    #cv2.imshow('q11', bottom_left_1)
    

#####################################################################################################################

# Set Serial Port

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)  # Linux/Mac port setting

# global value for sens distance
sen1 = 90 ## right
sen2 = 90
sen3 = 90
sen4 = 90 
sen5 = 90

ir_1_THRESHOLD = 100  # 0 : black 1 : white
ir_2_THRESHOLD = 100
ir_3_THRESHOLD = 100

obstacle_left = False
obstacle_center = False
obstacle_right = False
obstacle_little_right = False
obstacle_little_left = False
#obstacle_full = True

line = False

# global value
line_cnt = 0 # stop line scenario
line = False
flag = 1

traffic_1 = True  # first -> green light
traffic_2 = True  # second -> green light


# Sensor State
ir_1 = 0  # 0 -> black  1 -> white
ir_2 = 0
ir_3 = 0
# ir_4 = 0 

# son_ori = 0  # depth threshold 
# mode = 3




def read_serial_data():
    if ser.isOpen():
        try:
            linee = ser.readline().decode('utf-8').strip()  # 시리얼 데이터 읽기 및 디코딩
            if linee.startswith('*'):  # 올바른 데이터 라인인지 확인
                parse_sensor_data(linee)
        except Exception as e:
            print("Error reading from serial port: ", e)

def parse_sensor_data(linee):
    global ir_1, ir_2, ir_3
    global sen1, sen2, sen3, sen4, sen5
    try:
        parts = linee[1:].split(',')  # '*' 이후 값을 파싱
        if len(parts) != 8:
            print("Invalid data length:", len(parts))
            return
        ir_1 = int(float(parts[0].strip()))
        ir_2 = int(float(parts[1].strip()))
        ir_3 = int(float(parts[2].strip()))
        sen1 = int(float(parts[3].strip()))
        sen2 = int(float(parts[4].strip()))
        sen3 = int(float(parts[5].strip()))
        sen4 = int(float(parts[6].strip()))
        sen5 = int(float(parts[7].strip()))
    except Exception as e:
        print("Error parsing sensor data: ", e)

#def obstacle_check():  
def obstacle_update():
    global obstacle_left, obstacle_center, obstacle_right
    global obstacle_little_right, obstacle_little_left

    
    if sen2 <= 20 and sen3 <= 20 and sen4 <= 20:
        obstacle_center = True
        print("obstacle_center")

  
    if sen2 <= 20 and sen3 <= 20 and sen4 > 20:
        obstacle_left = True
        print("obstacle_right")

    if sen2 > 20 and sen3 <= 20 and sen4 <= 20: 
        obstacle_right = True
        print("obstacle_left")

    if sen2 <= 20 and sen3 > 20 and sen4 > 20:
        obstacle_little_right = True
        print("obstacle_little_right")

    if sen2 > 20 and sen3 > 20 and sen4 <= 20:
        obstacle_little_left = True
        print("obstacle_little_left")
        
def obstacle_forward_check():
   # global obstacle_left, obstacle_center, obstacle_right
    obstacle_update()
#    if obstacle_left == True or obstacle_center == True or obstacle_right == True: 
#        return True
#    else:
#        return False
    if sum([obstacle_left, obstacle_center, obstacle_right]) >= 2:
        return True
    else:
        return False
    
def ir_line_check():

    global ir_1
    global ir_2
    global ir_3

    global line

    if ir_1 < ir_1_THRESHOLD and ir_2 < ir_2_THRESHOLD and ir_3 < ir_3_THRESHOLD:

        line = True 

        return True 
    


def main():
    global frame
    global cap
    global flag
    global line_cnt
    global line
   # global ir_4
    global traffic_1, traffic_2
    global throt
    global mask
    global bottom_left_sum
    global bottom_left_sum_1
    
    throt = 0.2
    
    car.throttle = 0.0
    
    # Initial Value : line = false, line_cnt = 0 , flag = 1
    
    
    #line_input = input("Enter the line value(true,false) ") 

    #line = line_input.lower() in ["true", "yes", "1"]
    #line_cnt = int(input("Enter the line_cnt value(initial : 0) "))  # Convert line_cnt to an integer
    #flag = int(input("Enter the flag value(initial : 1) "))
    

    cap = cv2.VideoCapture(0)
    
 
    if not cap.isOpened() :
        print("asdf")
        return

      #  input(line_cnt)
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

        cv2.imshow('q1', bottom_left)
        cv2.imshow('q12', frame)
        cv2.imshow('q11', bottom_left_1)
        
        read_serial_data()
        print(f"\nCurrent flag: {flag}")
        
        if flag == 1:  
        
            drive_shit()  
        # if obstacle_forward_check() == True or ir_line_check() == True:  
            if ir_line_check() == True:
                flag = 0
            
                
        elif flag == 0:  # Stop
            # line_stop()
            line_cnt += 1  # Add when the stop line is met
            if line_cnt == 1:   # First
                if traffic_1 == False:  # Traffic light is red
                    time.sleep(3000) # Wait for 2 seconds
                flag = 2
            if line_cnt == 2:  # Second
                # line_stop()
                if traffic_2 == False:  # Traffic light is red
                    time.sleep(3000) # Wait for 2 seconds
                else:
                    line_stop()
                flag = 3
            
            if line_cnt == 3:
                if ir_line_check() == False:
                    line_stop()
            line = False  # Reset line state
        elif flag == 2:  # Child protection
            while True:
                drive_shit()
                if obstacle_forward_check() == True:
                    move_obstacle()
                    print("Passing dynamic obstacle")
                    flag = 1
                    break
                
    # elif flag == 3:  
        # Static obstacle
        # if obstacle_check() == True:
        #     silence_obstacle()
        #     print("Passing static obstacle")
        #     flag = 1
        # time.sleep(4000)
        # flag = 1
    # elif flag == 4:  # End
    # line_stop()
        elif flag == 3:
            print("jingjing")
        # When flag is 3, the state is stopped with line_stop() for jingjing
        # After jingjing, it proceeds
        # jingjing
            flag = 4
            
        else: # After jingjing, bypass the parking lot and go to the last stop line block
            flag = 1
            print("Ignoring the parking lot and driving to the final stop line.")






def move_obstacle():
    #global obstacle_left, obstacle_center, obstacle_right
    while True:
        if obstacle_forward_check() == True:
            line_stop()
        else:
            break
    print("move_obstacle")

# def silence_obstacle():
    
#    print("silence_obstacle")


cap.release()

cv2.destroyAllWindows()    


# main
if __name__ == "__main__":
    main()
