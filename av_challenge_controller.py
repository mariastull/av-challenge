"""av_challenge_controller controller."""

from controller import Camera
from vehicle import Driver


driver = Driver()

timestep = int(driver.getBasicTimeStep())

lidar = driver.getLidar('Sick LMS 291')
lidar.enable(timestep)
# print("Field of view", lidar.getFov())
# print("Dimension", lidar.getHorizontalResolution())

front_camera = driver.getCamera("front_camera")
# rear_camera = driver.getCamera("rear_camera")
front_camera.enable(10)
# rear_camera.enable(30)

        

# Adapted from Webots av example
def line_angle(x_min, x_max):
    width = front_camera.getWidth()
    height = front_camera.getHeight()
    fov = front_camera.getFov()

    image = front_camera.getImageArray()
    
    pixel_count = 0
    sumx = 0
    
    for x in range(x_min, x_max):
        for y in range (int (height/2), height): # look for white in bottom half of the image
            if (image[x][y][0] + image[x][y][1] + image[x][y][2] > 350 ) and (image[x][y][0] > 200 or image[x][y][1] > 200 or image[x][y][2] > 200):
            # if image[x][y][0] > 195 and image[x][y][1] > 195 and image[x][y][2] > 195:
                print x, y, image[x][y][0], image[x][y][1], image[x][y][2]
                sumx +=x
                pixel_count += 1

    print "avgs", min_color, max_color, float(sum_color)/num_pixels
    if pixel_count <= 1:
        return -1
    print(sumx, pixel_count, width, fov)
    return (float(sumx) / pixel_count)

def getLidarReading():
    image = lidar.getRangeImage()
    center_clear = 0
    right_clear = 0
    left_clear = 0
    for i in range(20, 79):
        if image[i] > 15 and image[i] < 80: 
            right_clear += 1
    for i in range(79, 100):
        if image[i] > 15: 
            center_clear += 1
    for i in range (100, 160):
        if image[i] > 15 and image[i] < 80: 
            left_clear += 1
    print "center", center_clear, "right", right_clear, "left", left_clear
    if center_clear > 18:
        return "center"
    if right_clear > left_clear:
        return "right"
    else:
        return "left"
        

   
def main():
    count = 0
    prev_width = 65
    min_x = 44
    max_x = 84
    PID_need_reset = False
    driver.setCruisingSpeed(50)
    driver.setGear(1)
    driver.setThrottle(1)
    prev_angle = 64
    
    while driver.step() != -1:
        count += 1
        if count % 10 == 0:
            print("speed", driver.getCurrentSpeed())
            angle = line_angle(min_x, max_x)
            print("angle", angle)
            if angle == -1: # can't find the line
                driver.setCruisingSpeed(1)
                if prev_angle == -1: # really can't find the line
                    driver.setCruisingSpeed(0) # come to a complete halt before lidar reading
                    for i in range(5):
                        driver.step()
                    print ("should be stopped", driver.getCurrentSpeed())
                # driver.setBrakeIntensity(.4)
                min_x = 0
                max_x = 128
                result = getLidarReading()
                if result == "center":
                    print("lidar clear ahead")
                    driver.setSteeringAngle(0)
                    driver.setCruisingSpeed(20)
                    # driver.setThrottle(.5)
                elif prev_angle == -1:
                    if result == "right":
                        print("lidar right")
                        driver.setSteeringAngle(.05)
                        driver.setCruisingSpeed(5)
                    if result == "left":
                        print("lidar left")
                        driver.setSteeringAngle(-.05)
                        driver.setCruisingSpeed(5)
            else:
                if (angle > 60 and angle < 68): # straightaway
                    min_x = 44
                    max_x = 84
                    if driver.getCurrentSpeed() < 30:
                        driver.setCruisingSpeed(50)
                        # driver.setThrottle(1)
                        driver.setSteeringAngle(0)
                else:
                    steering_angle = ((angle - 64) / 64 / (3.1415/2))
                    print("steering", steering_angle)
                    # driver.setBrakeIntensity(.4)
                    driver.setSteeringAngle(steering_angle)
                    # driver.setThrot tle(.8)
                    driver.setCruisingSpeed(25)
                    for i in range(21):
                        driver.step()
                    angle_after_turn = line_angle(min_x, max_x)
                    print(angle, angle_after_turn)
                    # if angle_after_turn == -1:
                        # driver.step()
                    # 
                    # if abs(angle_after_turn - 64) < abs(angle - 64):
                        # print "turn made things better"
                    # else:
                        # print "turn made things worse"
                    
                                            
            prev_angle = angle
            
            # if angle == -1:
                # PID_need_reset = True
            # steer_angle = applyPID(angle, PID_need_reset)
            # print (angle, steer_angle)
          
            # driver.setSteeringAngle(steer_angle)
     



main()
        
    # Process sensor data here.

    # Enter here functions t o send actuator commands, like:
     # motor.setPosition(10.0)
     # pass

# Enter here exit cleanup code.

