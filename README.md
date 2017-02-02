Missoula Robotics Team's 2017 code for _Steamworks_
---

A few notes about the code this year:

**Connections**:

- **MXP port** - ADIS16448 imu
- **PWM 0** - left motors (y-cable to the two ESCs)
- **PWM 1** - right motors (y-cable)
- **PWM 2** - ball launcher flywheel ESC
- **Analog in 0** - front rangefinder input
- **Analog in 1** - rear rangefinder input
- **Digital 0** - launcher encoder channel A
- **Digital 1** - launcher encoder channel B

**Control devices**:

- **USB HID id 0** - xBox/logitech controller
- **USB HID id 1** - arduino button panel
- **SmartDashboard** - choosing autonomous start location
- **Processing Dashboard** - connects over networktables, edits settings

**Controls**:

- **Both joystick vertical axes** - tank drive for the two sides of the robot
- **yes** - woof

**Button Panel Layout**:

- ...
