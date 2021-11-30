# Robotic Deep Rolling GUI

## Python packages
`python3 -m pip install general-robotics-toolbox numpy pyqt5 pyqt5-tools protobuf beautifulsoup4 ws4py pyyaml pandas openpyxl`

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

If you installed `pyqt5-tools` you will see Qt Designer at

`Python39\Lib\site-packages\qt5_applications\Qt\bin\designer.exe`

### `src > robot_kin`
Forward and inverse kinematics for ABB IRB-6640

### `src > rpi_abb_irc5`
Interface to ABB's EGM

Based on https://github.com/rpiRobotics/rpi_abb_irc5

Ported to python3, and added a function for Cartesian robot control

### `src > toolpath_gen`
Generate toolpath files for the controller to execute