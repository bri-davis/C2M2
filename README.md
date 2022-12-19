# C2M2 Robotics

Build a real-world vehicular traffic flow model! [Demo Video](https://tuprd-my.sharepoint.com/:v:/g/personal/tug75084_temple_edu/EYcL90TTsDBemm1bg_PlE2sBZ7Uv0Jka2aVn2Fg6zPKqow?e=jqCne1)

This repositiory hosts the robot controller code, robot build instructions, and previous experimental results and takeaways. Developed in conjunction with the [Center for Computational Mathematics and Modeling](https://c2m2.cst.temple.edu/). 


## Required Resources
For this long-term project, we are using the following version of [ev3dev](https://www.ev3dev.org/):

Image file: ev3dev-jessie-ev3-generic-2017-09-14  
Kernel version: 4.4.87-22-ev3dev-ev3  
Board: LEGO MINDSTORMS EV3 Programmable Brick  
Revision: 0006  
Brickman: 0.8.1  
ev3devKit: 0.4.3

All of the code in this repo is written in [Python](https://www.python.org/downloads/), however the ev3dev community hosts other languages as well.

## Getting Started
Will need a physical EV3 build kit, an SD card, and a gradient track to run the robot on.
The EV3 will need to need to be setup to run the open-source image of ev3dev using the instructions found [here](https://www.ev3dev.org/docs/getting-started/). The robot build we achieved can be found in the Robot Build folder.

Afterward, the Python scripts can be transferred directly to the EV3 brick and run via SSH.

''' python3 MostUpdatedPIDController.py '''

The robot will start following the gradient track. You may need to tune the hyperparameters for optimal line-following by referencing the PID_Controllers notebook.

## Future Considerations
- Transferring the robot ecosystem to the open-source [ROS](https://www.ros.org/) for better standardization / adoption
- Achieving reliable communication between robots using [MQTT](https://mqtt.org/) for increased accuracy
- Rewriting main script from Python to C++ in order to reduce time complexity (previously proven in a prototype)



