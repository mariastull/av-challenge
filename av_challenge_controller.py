"""av_challenge_controller controller."""

from controller import Camera
from vehicle import Driver
import warnings
warnings.filterwarnings("ignore")
import numpy as np

DEBUG_FLAG = True
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

# rear_camera = driver.getCamera("rear_camera")
# rear_camera.enable(30)

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

# Window: 80 pixels around center 
# used to look for white line if we think we know where it is
NARROW_XMIN = CAM_CENTER - 40
NARROW_XMAX = CAM_CENTER + 40

BACK_CAM_CENTER_MIN = BACK_CAM_CENTER - 3
BACK_CAM_CENTER_MAX = BACK_CAM_CENTER + 3

#for when we messed up a turn and are just trying to find the line again
LOST_XMIN = CAM_CENTER - 10
LOST_XMAX = CAM_CENTER + 10

#lidar for avoidance
LIDAR_LT = 60
LIDAR_RT = 115

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
    
    maxX = -1*float("inf")
    minX = float("inf")
    maxY = -1*float("inf")
    minY = float("inf")
   
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
                
                if x > maxX:
                    maxX = x
                if x < minX:
                    minX = x
                if y > maxY:
                    maxY = y
                if y < minY:
                    minY = y
            
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
        
    grey_count = 0
    tot = 0
    
    for i in range(minX, maxX+1):
        for j in range(1,6):
            try:
                #look @ [i][maxY+j]
                pix = image[i][maxY+j]
                #is it grey?
                if (abs(pix[0]-pix[1]) <= 15) and (abs(pix[0]-pix[2]) <= 15) and (abs(pix[1]-pix[2]) <= 15) :
                    grey_count += 1
                tot += 1
            except:
                pass
            try:
                #look @ [i][minY-j]
                pix = image[i][minY-j]
                #is it grey?
                if (abs(pix[0]-pix[1]) <= 15) and (abs(pix[0]-pix[2]) <= 15) and (abs(pix[1]-pix[2]) <= 15) :
                    grey_count += 1
                    
                tot += 1
            except:
                pass
                
            
    for i in range(minY, maxY+1):
        for j in range(1,6):
            try:
                #look @ [minX-j][i]
                pix = image[minX-j][i]
                #is it grey?
                if (abs(pix[0]-pix[1]) <= 15) and (abs(pix[0]-pix[2]) <= 15) and (abs(pix[1]-pix[2]) <= 15) :
                    grey_count += 1
                tot += 1
            except:
                pass
            try:
                #look @ [maxX+j][i]
                pix = image[maxX+j][i]
                #is it grey?
                if (abs(pix[0]-pix[1]) <= 15) and (abs(pix[0]-pix[2]) <= 15) and (abs(pix[1]-pix[2]) <= 15) :
                    grey_count += 1
                    
                tot += 1
            except:
                pass
                
    
    if DEBUG_FLAG:
        print("total white pixels:", pixel_count, "avg x coordinate: ", sumx/pixel_count)
        print("percent surrounding that are grey:", float(grey_count)/tot)
    
    if float(grey_count)/tot < 0.6:
        #print('BAD LINE')
        return -1
    # Otherwise return the average x-coordinate of the white pixels 
    return (float(sumx) / pixel_count)
    
def objDetect():

    image = front_camera.getImageArray()
    
    pixel_count = 0
    sus_count = 0
    sumx = 0
    sumy = 0
   
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
                sumy += y
        
    
    # We haven't found white pixels
    if sus_count <= 1: 
        return -1
    
    if DEBUG_FLAG_OBJ:
        print("total sus pixels:", sus_count, "avg x coordinate: ", sumx/sus_count)
    
    # Otherwise return the average x-coordinate of the white pixels 
    return (float(sumx) / sus_count), (float(sumy) / sus_count);
    
def getLidarReading(thresh):
    image = lidar.getRangeImage()
    center_clear = 0
    right_clear = 0
    left_clear = 0
    
    pings = []
            
    for i in range(LIDAR_LT, LIDAR_RT):
        if image[i] < thresh:
            pings.append(i)
            
    return pings

def getLidarReadingOld():
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

def killSpeed():
    driver.setCruisingSpeed(-120)
    while driver.getCurrentSpeed() > 0:
        print("slowing down; speed:", driver.getCurrentSpeed())
        for i in range(5):
            driver.step()

def reverse():
    print("Reverse straight back")
    killSpeed()
    for i in range(100):
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
            back_angle = BACK_CAM_CENTER
        
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
    # We've just started; who knows what our acceleration should be?
    # Maybe someone who knows anything about physics
    if count < 10:
        return False
    
    # Big suddden crash -- this is probably? a crash even if we applied the brakes
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
    
        # This isn't sudden, we were already slowing down
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
   
def killSpeedFwd():
    driver.setCruisingSpeed(50)
    while driver.getCurrentSpeed() < 0:
        for i in range(5):
            driver.step()
    driver.setCruisingSpeed(0)
            
def lookForRoad():
    image = front_camera.getImageArray()
    
    pixel_count = 0
    sumx = 0
   
    clear = {}
    for i in range (0, CAM_WIDTH):
        clear[i] = True
    
    # Only looking at bottom 1/3 of image 
    # where the road is likely to be and we have decent image fidelity
    for y in range (int (2*CAM_HEIGHT/3), CAM_HEIGHT):
        
        for x in range(0, CAM_WIDTH):
            
            # Is this pixel black?
            if  image[x][y][0] < 75 and image[x][y][1] < 75 and image[x][y][2] < 75:
                # print("black pixel at x:", x, "y:", y, "RGB:", image[x][y][0], image[x][y][1], image[x][y][2])
                sumx +=x
                pixel_count += 1
            elif (image[x][y][0] + image[x][y][1] + image[x][y][2] > 350 ) and (image[x][y][0] > 200 or image[x][y][1] > 200 or image[x][y][2] > 200):
                # white pixel
                sumx += x
                pixel_count += 1
            else:
                clear[x] = False
                # print("non-black pixel at x:", x, "y:", y, "RGB:", image[x][y][0], image[x][y][1], image[x][y][2])
    
    longest_clear_range = 0
    min = 0
    max = 0
    best_min = 0
    best_max = 0
    in_range = True
    for x in range(0, CAM_WIDTH):
        if clear[x]:
            if in_range:
                max = x
            else:
                min = x
                max = x
                in_range = True
        else:
            if in_range:
                in_range = False
                if max - min > best_max - best_min:
                    best_min = min
                    best_max = max
                print ("clear range:", min, max)
    print("longest clear range: ", best_min, best_max)
    return (best_max + best_min) / 2.0


def lidarObjectAhead():
    image = lidar.getRangeImage()
    distance = 0
    
    obj_ahead = 0
    # 79-100 degrees: center of image
    for i in range(85, 96):
        if image[i] < 8: 
            obj_ahead += 1
            distance += image[i]
        print("lidar reading", i, image[i])
    
    if obj_ahead > 1:
        return (True, (distance / obj_ahead))
    return (False, 0)

def sillyObjectAvoidance():
    print("Beginning silly object avoidance")
    
    lidar_vals = lidarObjectAhead()
    
    if lidar_vals[1] > 3.5:
        driver.setSteeringAngle(0)
        driver.setCruisingSpeed(30)
        for i in range(10):
            driver.step()
    
    steering_angle = 0
    angle = lookForRoad()
    
    if abs(angle) < 1:
        angle = CAM_CENTER
    
    if angle < CAM_CENTER:
        print ("left")
        steering_angle = -.33
    elif angle > CAM_CENTER:
        print("right")
        steering_angle = .33
        
    driver.setSteeringAngle(steering_angle)
    print("angle", angle, "Steering angle:", driver.getSteeringAngle())
    
    driver.setCruisingSpeed(30)
    
    for i in range(55):
        driver.step()
    
    print("angle", angle, "Steering angle:", driver.getSteeringAngle())
    
    driver.setSteeringAngle(.5 * steering_angle)
    max_steps = 10 * int(lidar_vals[1])
    if max_steps > 40:
        max_steps = 40
    for i in range(max_steps):
        driver.step()
    
    driver.setSteeringAngle(-.75 * steering_angle)
    for i in range(25):
        driver.step()
    
    driver.setSteeringAngle(0)
  
    for i in range(50):
        driver.step()
        
    print ("Done with silly object avoidance")


def main():

    count = 0
    
    min_x = NARROW_XMIN
    max_x = NARROW_XMAX
    turning = False
    
    driver.setCruisingSpeed(60)

    x_coord = CAM_CENTER
    prev_x_coord = CAM_CENTER
    obj_xcors = []
    obj_ycors = []
    last_coords = [CAM_CENTER, CAM_CENTER, CAM_CENTER, CAM_CENTER]
    
    prev_accel_vals = accelerometer.getValues()
    
    frames = []

    
    while driver.step() != -1:
          
        count += 1
        
        
        
        accel_vals = accelerometer.getValues()
        # print(accel_vals, driver.getTargetCruisingSpeed(), driver.getBrakeIntensity(), count)
        
        # Crash detection
        if suddenStop(count, accel_vals, prev_accel_vals):
            print("CRASH!?!?", accel_vals)
            killSpeed()
            back_camera.enable(10)
            if backCamAngle() != -1:
                reverseWithBackup()
            else:
                reverse()
            back_camera.disable()
            killSpeedFwd()
        
            if lidarObjectAhead()[0]:
                print("Object ahead!")
                killSpeedFwd()
                sillyObjectAvoidance()
        
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
            
            last_coords.append(x_coord)
            
            last_coords = last_coords[1:]
            
            #print(x_coord)
            
            if DEBUG_FLAG:
                print("speed", driver.getCurrentSpeed())
                print("x-coord", x_coord)
                print("last coords", last_coords)
                
                
            curr_angle = driver.getSteeringAngle()
            print(sum(last_coords), len(last_coords), curr_angle)
            if (sum(last_coords) == len(last_coords)*-1) and (abs(curr_angle) > 0.05) and (abs(curr_angle) < 0.1):
                
                print('should be turning harder here')
                print(curr_angle, turning)
                driver.setSteeringAngle(curr_angle * 5)
                new_angle = line_angle(0, CAM_WIDTH)
                while not(new_angle < LOST_XMAX and new_angle > LOST_XMIN): #still havent found line
                    print(new_angle, 'is not less than', LOST_XMAX, 'and greater than', LOST_XMIN)
                    driver.step()
                    new_angle = line_angle(0, CAM_WIDTH)
                    print('still looking for the line!!!')
 
                print('found line!!!', new_angle)
                driver.setSteeringAngle(0)
                for i in range(10):
                    driver.step()
                    print('going fwd now')
                        
                x_coord = line_angle(min_x, max_x)
            elif (sum(last_coords) == len(last_coords)*-1) and abs(curr_angle) > 0.1 and abs(curr_angle) < 0.3:
                driver.setSteeringAngle(1.5 * curr_angle)
                for i in range(15):
                    driver.step()
                driver.setSteeringAngle(curr_angle)
                
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
                    #print('maybe narrowing is bad?')
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
            
        h_obj, v_obj = objDetect()
        # print(h_obj)
        
        if h_obj == -1:
            obj_xcors = []
            obj_ycors = []
        else:
            obj_xcors.append(h_obj)
            obj_ycors.append(v_obj)
            
        if (len(obj_xcors) >= 8) and (turning == False):
            #check that all xcors are increasing or decreasing
            if ((obj_xcors == sorted(obj_xcors)) or (obj_xcors == sorted(obj_xcors, reverse=True))) and (len(obj_xcors) == len(set(obj_xcors))):
                if DEBUG_FLAG:
                    print("object detected")
                    #should be an object there!!
                    print(obj_xcors)
                #horizontally moving    
                if max(obj_ycors) - min(obj_ycors) <= 6:
                    driver.setCruisingSpeed(0)
                    driver.setBrakeIntensity(1)
                    while h_obj != -1:
                        driver.step()
                        h_obj = objDetect()
                        
                #vertically moving
                else:
                    sillyObjectAvoidance()
                    
                obj_xcors = []
                obj_ycors = []
       
            
     



main()
        
    # Process sensor data here.

    # Enter here functions t o send actuator commands, like:
     # motor.setPosition(10.0)
     # pass

# Enter here exit cleanup code.