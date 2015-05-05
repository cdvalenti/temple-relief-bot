# Temple Relief Robot
-----
Code repository for Temple University Senior Design project *"Temple Relief Robot"*

**Project website**: https://sites.google.com/a/temple.edu/temple-relief-robot/

**Description**: Used AVR-C to control all aspects of robot control including:
* Power distribution
* Handheld controls 
* Digital filters to smooth out user input
* Tank tread drive-train motor control
* Closed-loop position control on high torque lifting arms
* Servo actuators for bridge release and grain delivery
    
**Directory**:

* **drive_buttons**: contains code for drive control and servos actuated by push buttons (ATMEGA328)
* **arm_control**: contains code for closed loop position feedback control for lifting arms (ATTINY84)
* **tests**: includes various test software for the design process


