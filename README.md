# mte380
## Background
This repository holds the software developed for MTE380, our third-year design project course. In this iteration of the course, the primary objective was to develop a robot to successfully navigate a 2m x 2m course comprised of various types of terrain. 

Below is a gif of our robot's performance:

[add gif here]

## Hardware

The microcontroller used on the robot is a STM32 NUCLEO-F446RE. 

The following sensors were used to determine the robot's position in the course: 
- Three [VL53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) time-of-flight (ToF) sensors, one forward-facing and two left-facing.
- An [ICM20948](https://www.adafruit.com/product/4554) 9-DoF inertial measurement unit (IMU) to detect the robot's rotation. 
- A downward-facing photoresistor to measure the colour of the terrain below.

The robot was moved by controlling four [brushed DC micro metal gearmotors](https://www.pololu.com/product/1101) using two [L298N](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf) motor controllers.

## Software
The embedded C written to govern the robot's movement can be found in `mte380/stm32`. It comprises the following modules:
| Module | Filename | Function |
| ---    | ---      | ---         |
| main | main.c | Driver initialization and superloop calling course correction and position determination functions.
| IMU Tracking | imu_tracking.c | Functionality to calculate the robot's rotation about the x-axis by numerically integrating each rotational veloicty reading. Used to determine whether the robot's chassis is parallel to the ground, which can affect ToF readings.
| Motor Controller Driver | l298n_motor_controller.c | Motor controller driver.
| Logger | logger.c | Creates a large buffer to store logs to and outputs logs over serial when requested.
| Movement | movement.c | Governs acceleration, enging braking, and turning.
| Photoresistor Driver | photoresistor.c | Uses an ADC to estimate the photoresistor's resistance and evaluate the type of surface the robot is above.
| Positioning | position.c | Reads from the side ToFs and uses them to make slight corrections to keep the robot heading straight. Reads from the front ToF to assess when to stop and turn. 
| Position Tracking | position_tracking.c | Very similar to `imu_tracking.c`. Tracks readings from the front-facing ToF to attempt to discard erroneous readings. Currently unused.

### Credits
The driver used for the VL53L0X ToF sensor can be found [here](https://github.com/lamik/VL53L0X_API_STM32_HAL). It is designed primarily for taking single measurements with polling, but we are using the ToF sensor in continuous ranging mode.

The driver used for the ICM20948 IMU can be found [here](https://github.com/mokhwasomssi/stm32_hal_icm20948).


