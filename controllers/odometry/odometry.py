"""odometry controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math


def run_robot(robot):
    timestep = 64  # get the time step of the current world.
    speed = 6.28

    # Motor Initialization
    wheels = []
    wheelsNames = ['wheel1', 'wheel2']
    for i in range(2):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)

    # Encoder Initializations
    left_en = robot.getDevice("encoder1")
    right_en = robot.getDevice("encoder2")
    left_en.enable(timestep)
    right_en.enable(timestep)

    # UltraSense Initializations
    ultraSense = []
    ultraSenseNames = ["ds_right", "ds_left", "ds_front",
                       "ds_left_left_front", "ds_left_front_front"]
    for i in range(5):
        ultraSense.append(robot.getDevice(ultraSenseNames[i]))
        ultraSense[i].enable(timestep)

    # Variable Initializations
    en_val = [0, 0]
    dist_val = [0, 0]
    array_loc = [1, 1]
    array_size = 0.24  # 2*robot_radius
    wheelCircum = 2*3.14*0.04  # 2*pi*wheel_radius
    turning_radius = 0.12  # distance between wheels
    en_unit = wheelCircum/speed  # speed in no of revolutions/sec(here 1)
    prev_en_val = [0, 0]
    bot_pos = [0, 0, 0]  # x,y, theta
    distance_threshold = 3
    turn_sensitivity = 2

    # Odometry
    while robot.step(timestep) != -1:
        en_val[0] = left_en.getValue()
        en_val[1] = right_en.getValue()

        # encoder data extraction
        for i in range(2):
            diff = en_val[i] - prev_en_val[i]
            if diff < 0.001:
                diff = 0
                en_val[i] = prev_en_val[i]
            dist_val[i] = diff * en_unit

        # Bot Position Computation
        bot_v = (dist_val[0] + dist_val[1])/2.0
        bot_w = (dist_val[0] - dist_val[1])/turning_radius

        dt = 1
        bot_pos[2] += bot_w * dt

        vx = bot_v*math.cos(bot_pos[2])
        vy = bot_v*math.sin(bot_pos[2])

        bot_pos[1] += vy * dt
        array_loc[1] = round(bot_pos[1]/array_size) + 1
        bot_pos[0] += vx * dt
        array_loc[0] = round(bot_pos[0]/array_size) + 1

        # print(f"x_arr: {array_loc[1]} y_arr: {array_loc[0]}")
        print(f"x: {bot_pos[1]} y: {bot_pos[0]} angle:{bot_pos[2]*180/3.14}")

        # Wall Following
        front_wall = ultraSense[2].getValue() < 10*distance_threshold
        left_wall = ultraSense[1].getValue() < distance_threshold

        if front_wall and left_wall:  # bot at a corner
            wheels[0].setVelocity(speed)
            wheels[1].setVelocity(-speed)
            for i in range(2):
                prev_en_val[i] = en_val[i]
        if not front_wall and left_wall:  # bot following a wall
            for i in range(2):
                wheels[i].setVelocity(speed)
                prev_en_val[i] = en_val[i]
        if front_wall and not left_wall:  # bot facing obstacle
            wheels[0].setVelocity(speed)
            wheels[1].setVelocity(speed/turn_sensitivity)
            for i in range(2):
                prev_en_val[i] = en_val[i]
        if not front_wall and not left_wall:  # bot away from wall
            wheels[0].setVelocity(speed/turn_sensitivity)
            wheels[1].setVelocity(speed)
            for i in range(2):
                prev_en_val[i] = en_val[i]

        # Wall Mapping
        rows, cols = (6, 6)
        wallmap = [[0]*cols]*rows
        wallmap[0][0] = 1
        angle = bot_pos[2]*180/3.14
        front_sense = ultraSense[2].getValue() > array_size
        left_sense = ultraSense[1].getValue() > array_size
        right_sense = ultraSense[0].getValue() > array_size
        if(angle < 90.0):
            if front_sense:
                wallmap[array_loc[1]][array_loc[0]+1] = 1  # x is 1, y is 0
            if left_sense:
                wallmap[array_loc[1]-1][array_loc[0]] = 1
            if right_sense:
                wallmap[array_loc[1]+1][array_loc[0]] = 1

        print(
            f"right: {ultraSense[0].getValue()}, left: {ultraSense[1].getValue()}, front: {ultraSense[2].getValue()}")
        # print(wallmap)
        pass


if __name__ == "__main__":
    # create the Robot instance.a
    robot = Robot()
    run_robot(robot)
