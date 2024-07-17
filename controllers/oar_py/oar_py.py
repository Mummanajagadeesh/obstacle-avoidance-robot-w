"""oar_py controller."""


from controller import Robot

TIMESTEP = 32
MAX_SPEED = 6.28


def run_robot(robot):

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
  
    list_ps = []
    for ind in [0, 1, 2, 5, 6, 7]:
        sensor_name = 'ps' + str(ind)
        list_ps.append(robot.getDevice(sensor_name))
        list_ps[-1].enable(TIMESTEP)
    
    
    while robot.step(TIMESTEP) != -1:
    
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        

        for ps in list_ps:
            ps_val = ps.getValue()
            if ps_val > 100:
                left_speed = -MAX_SPEED
   
  
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)