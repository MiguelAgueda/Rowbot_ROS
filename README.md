# What is Ro*w*bot_ROS?

Ro*w*bot_ROS is an implementation of [Ro*w*bot](https://github.com/MiguelAgueda/Rowbot) 
which utilizes ROS for communicating data across multiple nodes.
In its current stage, Ro*w*bot is small rover built upon the Wild Thumper development platform.

## Components Used

| Component | Hardware Utilized | Usage |
|:-:|:-:|:-:|
| Main Computer | NVidia Jetson Nano | General processing, ROS, and LiDAR communication. |
| Micro Controller | Teensy 4.0 | IMU, GNSS, and ROS communication. |
| Micro Controller | Arduino Pro Micro | Radio communication. |
| LiDAR Sensor | RPLiDAR A2 | 2D rangescan measurements. |
| IMU | ICM-20948 9-DOF IMU | Accelerometer, gyroscope, and heading measurements. |
| GNSS | SAM-M8Q GNSS Module | Global position measurements. |
| Radio | nRF24L01 Radio Module | Remote control operation in locations without wireless connectivity. |
