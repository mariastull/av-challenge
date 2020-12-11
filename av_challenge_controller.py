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

accelerometer = driver.getAccelerometer("accelerometer")
accelerometer.enable(timestep)

# accelerometer = driver.getAccelerometer("gyro")
# accelerometer.enable(timestep)
# print(type(accelerometer))


front_camera = driver.getCamera("front_camera")
front_camera.enable(10)
#front_camera.recognitionEnable(10)

back_camera = driver.getCamera("rear_camera")
# back_camera.enable(30)

CAM_WIDTH = front_camera.getWidth()
CAM_HEIGHT = front_camera.getHeight()
CAM_CENTER = int(CAM_WIDTH/2)

BACK_CAM_WIDTH = back_camera.getWidth()
BACK_CAM_HEIGHT = back_camera.getHeight()
BACK_CAM_CENTER = int(BACK_CAM_WIDTH/2)

# Window: 8 pixels either side of center of frame 
# used to detect if white line is in center of camera view
CAM_CENTER_MIN = CAM_CENTER - 3
CAM_CENTER_MAX = CAM_CENTER + 3

BACK_CAM_CENTER_MIN = BACK_CAM_CENTER - 3
BACK_CAM_CENTER_MAX = BACK_CAM_CENTER + 3

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
   
    for x in range(0, CAM_WIDTH):
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


# Same as line_angle, but for the backup cam
def backCamAngle():
    image = back_camera.getImageArray()
    
    pixel_count = 0
    sumx = 0
    
    green_sumx = 0
    green_count = 0
   
    for x in range(0, BACK_CAM_WIDTH):
        # Only looking at bottom 1/3 of image 
        # where the road is likely to be and we have decent image fidelity
        for y in range (int (2*BACK_CAM_HEIGHT/3), BACK_CAM_HEIGHT):
            
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
    if green_count > 15:
        if DEBUG_FLAG:
            print("Green behind:", float(green_sumx)/green_count)
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
                if DEBUG_FLAG_OBJ:
                    print("sus pixel at x:", x, "y:", y, "RGB:", image[x][y][0], image[x][y][1], image[x][y][2])
                
                sus_count += 1
                sumx +=x
        
    
    # We haven't found white pixels
    if sus_count <= 1: 
        return -1
    
    if DEBUG_FLAG_OBJ:
        print("total sus pixels:", sus_count, "avg x coordinate: ", sumx/sus_count)
    
    # Otherwise return the average x-coordinate of the white pixels 
    return (float(sumx) / sus_count)

def getLidarReading():
    image = lidar.getRangeImage()
    center_clear = 0
    right_clear = 0
    left_clear = 0
    
    # 0-79 degrees: right side of image
    for i in range(0, 20):
        if image[i] > 15 and image[i] < 80: 
            right_clear += 1
            print("right clear", image[i])
        else:
            print("right obstacle", image[i])
    # 79-100 degrees: center of image
    for i in range(85, 96):
        if image[i] > 15: 
            center_clear += 1
            print("center clear", image[i])
        else:
            print("center obstacle", image[i])
    
    # 100 to 160 degrees: left side of image
    for i in range (160, 180):
        if image[i] > 15 and image[i] < 80: 
            left_clear += 1
            print("left clear", image[i])
        else:
            print("left obstacle", image[i])
            
    if DEBUG_FLAG:
        print ("lidar results: center", center_clear, "right", right_clear, "left", left_clear)
    if center_clear > 8:
        return "center"
    if right_clear > left_clear:
        return "right"
    else:
        return "left"

def killSpeed():
    while driver.getCurrentSpeed() > 0:
        print ("trying to go backwards")
        driver.setCruisingSpeed(-120)
        print("speed:", driver.getCurrentSpeed())
        driver.step()

# Backs up without changing wheel angles
def reverse():
    print("Reverse straight back")
    killSpeed()
    for i in range(150):
        driver.setCruisingSpeed(-20)
        driver.step()

def reverseWithBackup():
    print("Using back-up cam")
    back_camera.enable(10)
    killSpeed()
    driver.setCruisingSpeed(-20)
    for i in range(50):
        driver.step()
    for i in range(10):
        back_angle = backCamAngle()
        
        if back_angle == -1:
            reverse()
            return
        
        if back_angle > BACK_CAM_CENTER_MIN and back_angle < BACK_CAM_CENTER_MAX:
            driver.setSteeringAngle(0)
        
        else:
            # *0.75 to dampen the turn, *-1 because we're in reverse
            steering_angle = -.75 * ((back_angle - BACK_CAM_CENTER) / (BACK_CAM_WIDTH/2) / (3.1415/2))
            driver.setSteeringAngle(steering_angle)
        
        driver.setCruisingSpeed(-30)
        
        for i in range(5):
            for i in range(5):
                driver.step()
            new_angle = backCamAngle()
            if new_angle < BACK_CAM_CENTER_MAX and new_angle > BACK_CAM_CENTER_MIN:
                if DEBUG_FLAG:
                    print("Ending the turn early.")
                break
        
        if line_angle(0, 256) != -1:
            break
    back_camera.disable()

def suddenStop(count, accel_vals, prev_accel_vals):
    # This is probably just athe initial startup
    if count < 10:
        return False
    
    if accel_vals[0] < -20:
        if abs(accel_vals[0] - prev_accel_vals[0]) < 6:
            return False
        return True
    
    if accel_vals[1] < -20:
        if abs(accel_vals[1] - prev_accel_vals[1]) < 6:
            return False
        return True
    
    if driver.getBrakeIntensity() > .9:
        return False
    
    # we have slowed down a lot!
    
    if accel_vals[0] < -5:
    
        # This isn't sudden
        if abs(accel_vals[0] - prev_accel_vals[0]) < 6:
            return False
        
        # We just stopped braking; momentum may be carrying us backwards
        if driver.getBrakeIntensity() < .1 and prev_accel_vals[3] == 1:
            return False
            
        # We just started braking; ignore 
        if driver.getBrakeIntensity() > .1 and prev_accel_vals[3] == 0:
            return False
        
        return True

    if accel_vals[1] < -5:
        if abs(accel_vals[1] - prev_accel_vals[1]) < 6:
            return False
        
        # We just stopped braking; momentum may be carrying us backwards
        if driver.getBrakeIntensity() < .1 and prev_accel_vals[3] == 1:
            return False
            
        # We just started braking; ignore 
        if driver.getBrakeIntensity() > .1 and prev_accel_vals[3] == 0:
            return False
        
        return True
        
    return False

def main():

    count = 0
    
    min_x = NARROW_XMIN
    max_x = NARROW_XMAX
    turning = False
    
    driver.setCruisingSpeed(70)

    x_coord = CAM_CENTER
    prev_x_coord = CAM_CENTER
    obj_xcors = []
    
    prev_accel_vals = accelerometer.getValues()

    
    while driver.step() != -1:
        count += 1
        
        # Crash detection
        accel_vals = accelerometer.getValues()
        print(accel_vals, driver.getTargetCruisingSpeed(), driver.getBrakeIntensity(), count)
        if suddenStop(count, accel_vals, prev_accel_vals):
        # count > 10 and ((driver.getBrakeIntensity() < .1 and prev_accel_vals[3] != 1) or driver.getBrakeIntensity() > .1 and prev_accel_vals[3] == 1)((accel_vals[1] < -5 and abs(accel_vals[1]-prev_accel_vals[1]) > 2) or accel_vals[0] < -5 and abs(accel_vals[0]-prev_accel_vals[0]) > 2): 
        # if ((accel_vals[1] < -5 or accel_vals[0] < -5) and count > 10 and driver.getBrakeIntensity() < .1) or (accel_vals[1] < -12 or accel_vals[0] < -12) and count > 10:
            print("CRASH!?!?", accel_vals)
            killSpeed()
            if line_angle(0, CAM_WIDTH) == -1:
                reverseWithBackup()
            else:
                reverse()
        prev_accel_vals = accel_vals
        if driver.getBrakeIntensity() > .1:
            prev_accel_vals.append(1)
        else:
            prev_accel_vals.append(0)
            
        # Next: can we see the white line?
        # No: use back-up cam to steer toward it in reverse
        # Yes: Probably we hit an obstacle; back up and initiate obstacle avoidance
        # (I may make the obstacle always purple so it's easy to find)
        # Either using the avoiding object coming toward you method
        # Or lidar
        # Or look for black --> empty road w/o obstacle      
               
            
        if count % 10 == 0:
            
            x_coord = line_angle(min_x, max_x)
            
            if DEBUG_FLAG:
                print("speed", driver.getCurrentSpeed())
                print("x-coord", x_coord)
                
                
            
            # can't find the line, slow down
            if x_coord == -1: 
                turning = False
                driver.setBrakeIntensity(0)
                # 
                #driver.setBrakeIntensity(.3)
                driver.setCruisingSpeed(30)
                
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
                    
                    driver.setCruisingSpeed(60)
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
                            
            prev_x_coord = x_coord
     
        h_obj = objDetect()
        # print(h_obj)
        
        if h_obj == -1:
            obj_xcors = []
        else:
            obj_xcors.append(h_obj)
            
        if (len(obj_xcors) >= 8) and (turning == False):
            #check that all xcors are increasing or decreasing
            if ((obj_xcors == sorted(obj_xcors)) or (obj_xcors == sorted(obj_xcors, reverse=True))) and (len(obj_xcors) == len(set(obj_xcors))):
                print("object detected")
                #should be an object there!!
                print(obj_xcors)
                driver.setCruisingSpeed(0)
                driver.setBrakeIntensity(1)
                while h_obj != -1:
                    driver.step()
                    h_obj = objDetect()
            
            
            #22.5 22.8 23.4444 24.0 24.3333
            
     



main()
        
    # Process sensor data here.

    # Enter here functions t o send actuator commands, like:
     # motor.setPosition(10.0)
     # pass

# Enter here exit cleanup code.
