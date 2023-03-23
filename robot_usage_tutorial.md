# How to use the ABB robot
1. The entire robotic-deep-rolling-gui repository is downloaded onto the desktop in the high bay. In the repository, double click on the batch file to open the GUI.

2. Close both gates around the robot and then come and click the yellow "gate safety reset" button near the computer.

3. Test that everything is working by jogging the robot. To do this, on the pendant, click on menu > jogging. Pick up the pendant, press down the dead man switch, and then you should be able to move the joystick to jog the robot.

4. If workpiece offset needs to be restored, jog the roller to a centimeter above where the start point of the roll should be. Record the world x, y and z coordinates at this point (the first time that we did this we found that these coordinates were x: 0.4285 m, y:  -1.312 m, z:  1.0547 m). Type those values into workpiece offset.

5. Tare the force-torque sensor by clicking the tare button twice on the GUI.

6. You are now ready to begin the roll. Initiate the roll by pressing the dead man switch on the pendant, pressing play on the pendant, and then pressing start on the GUI.

# How to restore our backup settings
How to restore our backup: We backed up our settings onto RobotStudio. These settings can be restored if someone else is using the robot but note that this is not the ideal way of restoring our settings. To restore our settings, on RobotStudio, "request write access", then click "grant" on the pendant, then click the arrow underneath "backup" on RobotStudio for restore backup, then select our backup "deep_rolling_day1."
