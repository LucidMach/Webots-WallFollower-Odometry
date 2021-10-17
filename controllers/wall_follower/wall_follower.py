"""wall_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robots(robot):
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    distance_threshold = 80
    rotate_sensitivity = 10

    # motor config
    left_motor = robot.getMotor("left wheel motor")
    right_motor = robot.getMotor("right wheel motor")
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0)
    right_motor.setPosition(float("inf"))
    right_motor.setVelocity(0)

    # ultrasonic sensor config
    ultra_sen = []
    for i in range(8):
        sensor_name = "ps"+str(i)
        ultra_sen.append(robot.getDevice(sensor_name))
        ultra_sen[i].enable(timestep)

    while robot.step(timestep) != -1:
        # Ultra Sen Values
        for i in range(8):
            print(f"US{i}: {ultra_sen[i].getValue()}")

        left_wall = ultra_sen[5].getValue() > distance_threshold
        left_corner = ultra_sen[6].getValue() > distance_threshold
        front_wall = ultra_sen[7].getValue() > distance_threshold

        left_speed = max_speed
        right_speed = max_speed

        if front_wall:
            print("R")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("F")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("L")
                left_speed = max_speed/rotate_sensitivity
                right_speed = max_speed
            if left_corner:
                print("Adjust Space")
                left_speed = max_speed
                right_speed = max_speed/rotate_sensitivity

        # Drive Motors
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        pass


if __name__ == "__main__":
    # create the Robot instance.
    e_puck = Robot()
    run_robots(e_puck)
