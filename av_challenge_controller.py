"""av_challenge_controller controller."""

from controller import Camera
from vehicle import Driver
import warnings
warnings.filterwarnings("ignore")
import numpy as np

DEBUG_FLAG = False
DEBUG_FLAG_OBJ = False

driver = Driver()

timestep = int(driver.getBasicTimeStep())

lidar = driver.getLidar('Sick LMS 291')
lidar.enable(10)

# accelerometer = driver.getAccelerometer("gyro")
# accelerometer.enable(timestep)
# print(type(accelerometer))


front_camera = driver.getCamera("front_camera")
front_camera.enable(10)
#front_camera.recognitionEnable(10)

# rear_camera = driver.getCamera("rear_camera")
# rear_camera.enable(30)

CAM_WIDTH = front_camera.getWidth()
CAM_HEIGHT = front_camera.getHeight()
CAM_CENTER = int(CAM_WIDTH/2)

# Window: 8 pixels either side of center of frame 
# used to detect if white line is in center of camera view
CAM_CENTER_MIN = CAM_CENTER - 8
CAM_CENTER_MAX = CAM_CENTER + 8

# Window: 80 pixels around center 
# used to look for white line if we think we know where it is
NARROW_XMIN = CAM_CENTER - 40
NARROW_XMAX = CAM_CENTER + 40

if DEBUG_FLAG:
    print("Camera dimensions", "width:", CAM_WIDTH, "height:", CAM_HEIGHT)
        

# Adapted from Webots AV example
# x_min and x_max defines which part of the image we're looking at
# (usually a band around the center, unless we lost the white line)
# Returns the average x-coordinate of white pixels
# or green pixels, if we're approaching the finish line
def line_angle(x_min, x_max, printall=False):

    image = front_camera.getImageArray()
    
    pixel_count = 0
    sumx = 0
    
    green_sumx = 0
    green_count = 0
   
    for x in range(x_min, x_max):
        # Only looking at bottom 1/3 of image 
        # where the road is likely to be and we have decent image fidelity
        for y in range (int (2*CAM_HEIGHT/3), CAM_HEIGHT):
            
            # Is this pixel white?
            if (image[x][y][0] + image[x][y][1] + image[x][y][2] > 350 ) and (image[x][y][0] > 200 or image[x][y][1] > 200 or image[x][y][2] > 200):
                if DEBUG_FLAG:
                    print("white pixel at x:", x, "y:", y, "RGB:", image[x][y][0], image[x][y][1], image[x][y][2])
                sumx +=x
                pixel_count += 1
            
            # Looking for green pixels (finish line)
            if image[x][y][1] > 200 and image[x][y][0] < 100 and image[x][y][2] < 100:
                if DEBUG_FLAG:
                    print("GREEN at x:", x, "y:", y ,"RGB:", image[x][y][0], image[x][y][1], image[x][y][2])
                green_sumx += x
                green_count += 1
    
    # If we saw a lot of bright green pixels, we are approaching the finish line
    # Aim for that
    if green_count > 30:
        if DEBUG_FLAG:
            print("Green ahead:", float(green_sumx)/green_count)
        return float(green_sumx)/green_count
        
    
    # We haven't found white pixels
    if pixel_count <= 1: 
        return -1
    
    if DEBUG_FLAG:
        print("total white pixels:", pixel_count, "avg x coordinate: ", sumx/pixel_count)
    
    # Otherwise return the average x-coordinate of the white pixels 
    return (float(sumx) / pixel_count)
    
def objDetect():

    image = front_camera.getImageArray()
    
    pixel_count = 0
    sus_count = 0
    sumx = 0
   
    for x in range(22, 113):
        # Only looking at bottom 1/3 of image 
        # where the road is likely to be and we have decent image fidelity
        for y in range (50, CAM_HEIGHT):
            
            # Is this pixel white?
            if (image[x][y][0] + image[x][y][1] + image[x][y][2] > 350 ) and (image[x][y][0] > 200 or image[x][y][1] > 200 or image[x][y][2] > 200):
                pixel_count += 1
                
            #is this pixel gray?
            elif (abs(image[x][y][0]-image[x][y][1]) <= 15) and (abs(image[x][y][0]-image[x][y][2]) <= 15) and (abs(image[x][y][1]-image[x][y][2]) <= 15) :
                pixel_count += 1
                
            else:
                #pixel is likely sus
                if DEBUG_FLAG:
                    print("sus pixel at x:", x, "y:", y, "RGB:", image[x][y][0], image[x][y][1], image[x][y][2])
                
                sus_count += 1
                sumx +=x
        
    
    # We haven't found white pixels
    if sus_count <= 1: 
        return -1
    
    if DEBUG_FLAG:
        print("total sus pixels:", sus_count, "avg x coordinate: ", sumx/sus_count)
    
    # Otherwise return the average x-coordinate of the white pixels 
    return (float(sumx) / sus_count)

def getLidarReading():
    image = lidar.getRangeImage()
    center_clear = 0
    right_clear = 0
    left_clear = 0
    
    # 0-79 degrees: right side of image
    for i in range(20, 79):
        if image[i] > 15 and image[i] < 80: 
            right_clear += 1
    # 79-100 degrees: center of image
    for i in range(79, 100):
        if image[i] > 15: 
            center_clear += 1
    
    # 100 to 160 degrees: left side of image
    for i in range (100, 160):
        if image[i] > 15 and image[i] < 80: 
            left_clear += 1
            
    if DEBUG_FLAG:
        print ("lidar results: center", center_clear, "right", right_clear, "left", left_clear)
    if center_clear > 18:
        return "center"
    if right_clear > left_clear:
        return "right"
    else:
        return "left"
        

   
def main():

    count = 0
    
    min_x = NARROW_XMIN
    max_x = NARROW_XMAX
    turning = False
    
    driver.setCruisingSpeed(70)

    prev_x_coord = CAM_CENTER
    obj_xcors = []

    
    while driver.step() != -1:
        count += 1
            
        if count % 10 == 0:
            # accel_vals = accelerometer.getValues()
            # print(accel_vals)
            
            x_coord = line_angle(min_x, max_x)
            
            if DEBUG_FLAG:
                print("speed", driver.getCurrentSpeed())
                print("x-coord", x_coord)
                
                
            
            # can't find the line, slow down
            if x_coord == -1: 
                turning = False
                # 
                driver.setBrakeIntensity(.3)
                driver.setCruisingSpeed(15)
                
                # Really lost: can't find the line
                #if prev_x_coord == -1: 
                     
                 #   driver.setBrakeIntensity(.7)
                  #  driver.setCruisingSpeed(1)
                   
                # Since we can't find the white line, widen search field
                min_x = 0
                max_x = CAM_WIDTH
                
            else:
                
                # White line is in the center of our camera; we're going the right direction
                if (x_coord > CAM_CENTER_MIN and x_coord < CAM_CENTER_MAX):
                    turning = False
                    # Narrow search field, since we know where the white line is 
                    min_x = NARROW_XMIN
                    max_x = NARROW_XMAX
                    
                    if driver.getCurrentSpeed() < 30:
                        driver.setCruisingSpeed(70)
                        # Stop turning
                        driver.setSteeringAngle(0)
                        driver.setBrakeIntensity(0)
                else:
                    turning = True
                    # We found the white line, and it's not in the center; we need to turn
                    min_x = 0
                    max_x = CAM_WIDTH
                    
                    # Calculate steering angle
                    # Distance from center / center
                    # Divided by pi/2, because a) we assume the line is in front of us
                    # and b) it makes turns more gentle, we don't want to turn the wheels 90 degrees
                    steering_angle = ((x_coord - CAM_CENTER) / (CAM_WIDTH/2) / (3.1415/2))
                    
                    if DEBUG_FLAG: 
                        print("turning", steering_angle)
                    
                    if abs(steering_angle) > .1:
                        driver.setBrakeIntensity(.99)
                        #driver.setBrakeIntensity(.8)
                        # print("hard brake")
                    driver.setSteeringAngle(steering_angle*0.75)
                    driver.setCruisingSpeed(30)
                    # if abs(steering_angle) < .1:
                        # driver.setCruisingSpeed(40)
                   
                    
                    # for i in range(25):
                        # driver.step()
                    
                    # Commit to a turn; don't try to readjust before making some progress through turn
                    # Important because camera image shifts a lot during turns
                    # Every five steps, check to see if we have centered the white
                    for i in range(5):
                        for i in range(5):
                            driver.step()
                        new_angle = line_angle(0, CAM_WIDTH)
                        if new_angle < CAM_CENTER_MAX and new_angle > CAM_CENTER_MIN:
                            if DEBUG_FLAG:
                                print("Ending the turn early.")
                            break
     
        h_obj = objDetect()
        
        if h_obj == -1:
            obj_xcors = []
        else:
            obj_xcors.append(h_obj)
            
        if (len(obj_xcors) >= 8) and (turning == False):
            #check that all xcors are increasing or decreasing
            if ((obj_xcors == sorted(obj_xcors)) or (obj_xcors == sorted(obj_xcors, reverse=True))) and (len(obj_xcors) == len(set(obj_xcors))):
                #should be an object there!!
                print(obj_xcors)
                driver.setCruisingSpeed(0)
                driver.setBrakeIntensity(1)
                while h_obj != -1:
                    driver.step()
                    h_obj = objDetect()
            prev_x_coord = x_coord
            
            #22.5 22.8 23.4444 24.0 24.3333
            
     



main()
        
    # Process sensor data here.

    # Enter here functions t o send actuator commands, like:
     # motor.setPosition(10.0)
     # pass

# Enter here exit cleanup code.