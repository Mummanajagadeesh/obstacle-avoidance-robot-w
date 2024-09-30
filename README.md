# Obstacle Avoidance Robot

This repository contains the simulation of an **Obstacle Avoidance Robot** that detects and avoids obstacles using basic proximity sensors. The robot uses a simple reactive strategy—if an obstacle is detected, it changes direction to avoid it. No advanced algorithms or PID controllers are used in this implementation.

## Demo

Click the image below to watch a demo video of the robot in action:

[![Watch the video](https://img.youtube.com/vi/O75YmInmh-U/0.jpg)](https://www.youtube.com/watch?v=O75YmInmh-U)

## How It Works

### Robot Design

- **Proximity Sensors**: The robot is equipped with six proximity sensors (`ps0`, `ps1`, `ps2`, `ps5`, `ps6`, `ps7`) that help it detect obstacles in its surroundings. These sensors return a value proportional to the distance between the sensor and an object.
- **Motors**: The robot uses two independent motors to control its left and right wheels. By adjusting the velocity of each wheel, the robot can move forward or turn based on sensor input.

### Obstacle Avoidance Strategy

- **Basic Movement**: The robot moves forward at a constant speed unless an obstacle is detected.
- **Obstacle Detection**: When any of the proximity sensors detect an object within a certain threshold (sensor value greater than 100), the robot reacts by changing its direction.
- **Reaction to Obstacles**: If an obstacle is detected by one of the sensors, the robot slows down one of its wheels (by reversing its speed) to turn away from the obstacle.

### Control Logic

1. The robot starts by moving forward with both wheels at maximum speed.
2. As it moves, the proximity sensors continuously check for obstacles.
3. If a sensor detects an obstacle (sensor reading > 100), the robot slows down or reverses one of its wheels to change direction and avoid the obstacle.
4. Once the obstacle is cleared, the robot resumes its forward motion.

## Code Explanation

Here is the controller code for the robot:

```python
"""oar_py controller."""

from controller import Robot

TIMESTEP = 32
MAX_SPEED = 6.28

def run_robot(robot):

    # Initialize the motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Initialize the proximity sensors
    list_ps = []
    for ind in [0, 1, 2, 5, 6, 7]:
        sensor_name = 'ps' + str(ind)
        list_ps.append(robot.getDevice(sensor_name))
        list_ps[-1].enable(TIMESTEP)
    
    # Main loop
    while robot.step(TIMESTEP) != -1:
    
        # Default speed for both motors
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        
        # Check proximity sensor values
        for ps in list_ps:
            ps_val = ps.getValue()
            if ps_val > 100:
                # If an obstacle is detected, reverse the left motor speed
                left_speed = -MAX_SPEED
   
        # Set the motor speeds
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
```

### Key Points:

- **Motor Control**: The left and right motors are controlled independently to allow the robot to move forward or turn. The motors are initialized with a speed of 0, and the maximum speed is set to `6.28` (the maximum velocity in radians per second).
  
- **Proximity Sensors**: The robot uses 6 proximity sensors (`ps0`, `ps1`, `ps2`, `ps5`, `ps6`, `ps7`), which detect nearby objects. These sensors provide values that increase as the robot approaches an obstacle.

- **Obstacle Avoidance Logic**: The robot checks the proximity sensor values in every timestep. If any of the sensors detect an obstacle (i.e., the sensor value exceeds 100), the robot adjusts its movement by reversing the left wheel, causing the robot to turn and avoid the obstacle.

- **Simple Reactive Control**: There are no complex algorithms like PID or A* in this robot. The control logic is purely reactive: the robot changes its movement only when an obstacle is detected.

## Installation and Usage

### Requirements

- **Webots**: Install the Webots robotics simulator from [here](https://cyberbotics.com/).
- **Python**: Make sure you have Python installed to run the robot controller.

### Steps to Run

1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/Mummanajagadeesh/obstacle-avoidance-robot-w.git
   cd obstacle-avoidance-robot-w
   ```
2. Open Webots and load the **obstacle_avoidance_robot.wbt** world file in the simulation folder.
3. Run the simulation and observe the robot's behavior as it avoids obstacles.

## Future Enhancements

- **Advanced Algorithms**: Implementing more advanced algorithms like A* or potential field-based navigation to improve obstacle avoidance.
- **PID Control**: Adding a PID controller for smoother and more precise obstacle avoidance.
- **Multiple Robots**: Testing the behavior of multiple robots avoiding obstacles in the same environment.
  
---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
