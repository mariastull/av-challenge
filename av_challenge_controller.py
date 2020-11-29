"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
# from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
# import controller
import sys

# print(controller.__file__)

print("Python version")
print (sys.version)


# from controller import DistanceSensor, Robot
from vehicle import Driver
# from controller import DistanceSensor





# create the Robot instance.
# robot = Robot()
# front_camera = robot.getCamera("front_camera")
# rear_camera = robot.getCamera("rear_camera")

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
# motor = robot.getMotor('motor')
# print(type(motor))
# motor.setPosition(float('inf'))
# motor.setVelocity(100)
# ds = robot.getLidar('Sick LMS 291')
# ds.enable(timestep)


driver = Driver()
driver.setCruisingSpeed(50)
lidar = driver.getLidar('Sick LMS 291')
timestep = int(driver.getBasicTimeStep())
lidar.enable(timestep)
# print("banana")

print(type(lidar))
print("Field of view", lidar.getFov())
print("Dimension", lidar.getHorizontalResolution())
print(timestep)


# front_camera.enable(30)
# rear_camera.enable(30)

RIGHT_MIN = 0
RIGHT_MAX = 20
DOWN_RIGHT_MIN = 20
DOWN_RIGHT_MAX = 40
MID_RIGHT_MIN = 40
MID_RIGHT_MAX = 60
F_RIGHT_MIN = 60
F_RIGHT_MAX = 80
CENTER_MIN = 80
CENTER_MAX = 100
F_LEFT_MIN = 100
F_LEFT_MAX = 120
MID_LEFT_MIN = 120
MID_LEFT_MAX = 140
DOWN_LEFT_MIN = 140
DOWN_LEFT_MAX = 160
LEFT_MIN = 160
LEFT_MAX = 179

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
count = 0
right_turn_count = 0
center_clear_count = 0
left_turn_count = 0 
while driver.step() != -1:
    count += 1
    # Read the sensors:
    # Enter here functions to read sensor data, like:
     # val = ds.getValue()
    if count % 20 == 0:
        print driver.getCurrentSpeed()
        val = lidar.getRangeImage()
        
        right_obstacle = False
        down_right_obstacle = False
        mid_right_obstacle= False
        front_right_obstacle = False
        center_obstacle = False
        front_left_obstacle = False
        mid_left_obstacle = False
        down_left_obstacle = False
        left_obstacle = False
        
        print("Reading")
        for i in range(CENTER_MIN, CENTER_MAX):
            if val[i] < 20:
                center_obstacle = True
        for i in range(RIGHT_MIN, RIGHT_MAX):
            if val[i] < 20:
                right_obstacle = True
        for i in range(DOWN_RIGHT_MIN, DOWN_RIGHT_MAX):
            if val[i] < 20:
                down_right_obstacle = True
        for i in range(MID_RIGHT_MIN, MID_RIGHT_MAX):
            if val[i] < 20:
                mid_right_obstacle = True
        for i in range(F_RIGHT_MIN, F_RIGHT_MAX):
            if val[i] < 20:
                front_right_obstacle = True
        for i in range(LEFT_MIN, LEFT_MAX):
            if val[i] < 20:
                left_obstacle = True
        for i in range(F_LEFT_MIN, F_LEFT_MAX):
            if val[i] < 20:
                front_left_obstacle = True
        for i in range(DOWN_LEFT_MIN, DOWN_LEFT_MAX):
            if val[i] < 20:
                down_left_obstacle = True
        for i in range(MID_LEFT_MIN, MID_LEFT_MAX):
            if val[i] < 20:
                mid_left_obstacle = True
        if center_obstacle:
            print("center obstacle detected.")
            # print val
        if right_obstacle:
            print("right obstacle detected.")
        if down_right_obstacle:
            print("down right obstacle detected")
        if mid_right_obstacle:
            print("mid right obstacle detected")
        if front_right_obstacle:
            print("front right obstacle detected.")
        if front_left_obstacle:
            print("front left obstacle_detected.")
        if mid_left_obstacle:
            print("mid left obstacle detected")
        if down_left_obstacle:
            print("down left obstacle detected")
        if left_obstacle: 
            print("left obstacle detected.")
        if right_turn_count > 3:
            print("Turning right")
            driver.setSteeringAngle(.25) 
            driver.setCruisingSpeed(10)
            if not center_obstacle or not front_left_obstacle:
                print("no center obstacle")
                # print val
                center_clear_count += 1
                if center_clear_count > 1:
                    right_turn_count = 0
                    driver.setSteeringAngle(0)
                    driver.setCruisingSpeed(30)
                continue
            elif center_obstacle:
                print("still a center obstacle")
                continue
        if left_turn_count > 3:
            print("Turning left")
            driver.setSteeringAngle(-.25) 
            driver.setCruisingSpeed(10)
            if not center_obstacle or not front_right_obstacle:
                print("no center obstacle")
                # print val
                center_clear_count += 1
                if center_clear_count > 1:
                    left_turn_count = 0
                    driver.setSteeringAngle(0)
                    driver.setCruisingSpeed(30)
            elif center_obstacle:
                print("still a center obstacle")
                continue
        if center_obstacle:
            print("slowing down for center obstacle.")
            # print val
            driver.setCruisingSpeed(5)
        if center_obstacle and (not left_obstacle or not down_left_obstacle or not mid_left_obstacle or not front_left_obstacle):
            left_turn_count += 1
            right_turn_count = 0
        if center_obstacle and (not right_obstacle or not down_right_obstacle or not mid_right_obstacle or not front_right_obstacle):
            right_turn_count += 1
            left_turn_count = 0
            # driver.setSteeringAngle(.25) 
            # driver.setCruisingSpeed(10)
        # if not center_obstacle and driver.getCurrentSpeed()<5:
            # driver.setCruisingSpeed(20)
        # print(val[0], val[10], val[20], val[89], val[159], val[169], val[179])

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
     # motor.setPosition(10.0)
     # pass

# Enter here exit cleanup code.

