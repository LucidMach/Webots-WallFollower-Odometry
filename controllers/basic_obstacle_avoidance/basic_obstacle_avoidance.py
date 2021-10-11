from controller import Robot

TIME_STEP = 64
robot = Robot()

# Distance Sensors Initialization
ds = []
dsNames = ['ds_right', 'ds_left', 'ds_front']
for i in range(3):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

# Motors/Wheels Initialization
wheels = []
wheelsNames = ['wheel1', 'wheel2']
for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


def turnLeft():
    wheels[0].setVelocity(5)
    wheels[1].setVelocity(0)


# Distance to Front Wall
while robot.step(TIME_STEP) != -1:
    right = ds[0].getValue()
    left = ds[1].getValue()
    front = ds[2].getValue()

    if front > 24 and right > 1:
        for i in range(2):
            wheels[i].setVelocity(5)
    if front < 27 and right > 1:
        turnLeft()
