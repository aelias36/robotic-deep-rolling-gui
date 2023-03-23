# Robotic Deep Rolling GUI

A GUI for doing deep rolling using ABB industrial robots. 

[Simulator Installation](https://github.com/rpiRobotics/installation-tutorials/blob/main/robotstudio.md)

[GUI Installation](installation_tutorial.md)

[System Tuning (WIP)](tuning_tutorial.md)

[Toolpath Generation (WIP)](toolpath_generation_tutorial.md)

[GUI Usage (WIP)](GUI_usage_tutorial.md)

[How to use the ABB Robot](robot_usage_tutorial.md)

![GUI](https://user-images.githubusercontent.com/4022499/157300424-71651796-1adc-4792-b775-1491ed91696e.png)
![Robotic Deep Rolling](https://user-images.githubusercontent.com/4022499/157301229-466da7ed-f395-4ba4-be85-53e35356a963.png)


### Control goal:
Design and track a position / force profile

### Control structure:
* Generalized damper force control using force/torque sensor feedback, with filtering at sensor and robot
* Look-ahead control for robot delay compensation

### Implementation:
#### Parametric toolpath generation
* 16 parameters for geometry, force, and velocity
* Custom toolpath protocol designed for hybrid motion/force control

#### GUI for toolpath execution
* Interface with F/T sensor and robot controller
* Toolpath execution state machine covering multiple command types
* Designed for use with curved parts as well

#### High fidelity simulations for testing and tuning
* Connect to RobotStudio to simulate robot dynamics
* Linear spring model for joint flexibility modeling
* Can be expanded to bypass RobotStudio and use delay+LPF model for full robot simulation in one piece of software


---

## Sources

Older deep rolling code https://github.com/ShuyoungChen/force_motion_control_physical

Reference for general robotics toolbox https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py

## Folder breakdown

### `rapid`
Code which runs on ABB robot controller

### `src > controller`
Executes commands, from the GUI or from the toolpath file

### `src > ft_sensor`
Reads in force/torque data from ATI sensor

This file is based on this python3 code https://github.com/hehonglu123/MARS_Sample_Return

That code is based on this python2 code https://github.com/rpiRobotics/rpi_ati_net_ft

Code has been modified to be nonblocking

### `src > ft_simulation`
Robot deflection simulation for simulating force/torque sensor readings when using RobotStudio

### `src > gui`
Qt GUI made in Qt Designer

If you install `pyqt5-tools` using `python -m pip install pyqt5-tools` you will see Qt Designer at

`Python39\Lib\site-packages\qt5_applications\Qt\bin\designer.exe`

### `src > robot_kin`
Forward and inverse kinematics for ABB IRB-6640

### `src > rpi_abb_irc5`
Interface to ABB's EGM

Based on https://github.com/rpiRobotics/rpi_abb_irc5

Ported to python3, and added a function for Cartesian robot control

### `src > toolpath_gen`
Generate toolpath files for the controller to execute
