# Robotic Deep Rolling GUI

## Python packages
pip install pyqt5 pyqt5-tools

## Sources

Older deep rolling code
https://github.com/ShuyoungChen/force_motion_control_physical

## Folder breakdown

### Controller
Executes commands, from the GUI or from the toolpath file
### ft_sensor
Reads in force/torque data from ATI sensor

This file is based on this python3 code https://github.com/hehonglu123/MARS_Sample_Return
That code is based on this python2 code https://github.com/rpiRobotics/rpi_ati_net_ft

### general_robotics_toolbox
Taken from https://github.com/rpiRobotics/general-robotics-toolbox
### gui
Qt GUI made in Qt Designer
### robot_kin
Forward and inverse kinematics for ABB IRB-6640
### rpi_abb_irc5
Interface to ABB's EGM
Taken from https://github.com/rpiRobotics/rpi_abb_irc5
### toolpath_gen
Generate toolpath files for the controller to execute