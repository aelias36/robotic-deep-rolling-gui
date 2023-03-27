# Safety
1. The dead man switch is the safeguard and the way to immediately stop the robot. You can also stop the robot by pressing the red emergency button on the pendant. If the robot begins to oscillate uncontrollably (you will hear this), if it is moving someplace that it shouldn't be, if it is going to hit an external object, or if it is leaving the workpiece, immediately stop the roll. Always keep your eye on the robot to make sure that none of the above are happening and so that in case they do, you are ready to stop the robot.

# How to use the ABB robot
1. The entire robotic-deep-rolling-gui repository is downloaded onto the desktop in the high bay. In the repository, double click on the batch file to open the GUI.

2. Turn the system to manual mode (if it is not already) by turning the key on the setup box to the left of the computers to the hand icon which symbolizes manual mode (see picture below). The gates around the robot do not need to be closed if the system is in manual mode because the safeguard is the user holding down the dead man switch.
![image](https://user-images.githubusercontent.com/123105763/228028979-44650eb5-00c2-43bc-8b17-0e203ec7d750.png)

3. On the pendant, navigate to "menu" > "Production Window" and then click "PP to Main" on the pendant. This resets the program by moving back to the top of the code, and the next time that you click play on the pendant, the robot will move to the home position.
![image](https://user-images.githubusercontent.com/123105763/228029616-81bad497-65d9-4818-81af-715b29b80d22.png)

4. Assuming that you have already found the start position (workpiece and tool offsets at the location where the roll should start) and entered these into a config file, load the config file. The values that you entered into the config file should now appear in the "Workpiece Offset" and "Tool Offset" menus towards the lower right of the GUI screen. Additionally, in the top middle of the GUI screen under the big "WORKPIECE" heading, the x, y and z coordinates should be zero. There is a bug where the y coordinate may not be zero until you very slightly jog the robot, but everything is fine once you jog the robot and the value goes to zero. 

5. Press "Tare FT Sensor" on the GUI twice in order to tare all force and torque values on the GUI.

6. Next, press "Load Toolpath" to load in your toolpath file.

7. Run a 0 Force roll just to make sure that the path looks correct and the robot orientations look correct. Do this by pressing the dead man switch on the pendant, pressing the start button on the pendant, changing the setting in the lower left of the GUI to "0 Force," and then clicking the green "START" button on the GUI. 

8. If the 0 Force roll looked good, you are now ready to run the real deep roll. Do this by changing the setting in the lower left of the GUI to "Toolpath," reloading the toolpath file in order to restart the program, pressing the dead man switch, pressing the start button on the pendant, and then clicking the green "START" button on the GUI. 

# If you need to find the new start position for the robot (if the vice or table move)
1. Press the dead man switch on the pendant, click the start button on the pendant, and then use the jogging buttons on the GUI screen to navigate the robot to 1 cm above the starting point. Make sure to use low jogging speeds (adjust the speed on the GUI) when you are close to the workpiece to ensure that you don't accidentally make contact with the workpiece.

2. Record the "WORLD" x, y, and z coordinates as they appear on the left side of the GUI screen. 
![image](https://user-images.githubusercontent.com/123105763/228032872-c294fc4a-4231-4cae-85e6-f9650da2f3b8.png)

3. You will need the following tool offsets (they control the rotations of the robot/orientation of the roller):  euler_deg [0.0, 180.0, -90.0].

4. Edit the config file by entering the "WORLD" x, y, and z coordinates that you wrote down as the workpiece offsets, as shown in the image below. The tool offsets (angles) should already be saved in the config file assuming that these will stay the same from roll to roll.
![image](https://user-images.githubusercontent.com/123105763/228033770-1c0ac236-335e-4383-af1e-faa0ccaa44c4.png)

5. The config file is now ready to be loaded into the GUI.

# Troubleshooting - if connections don't seem to be working
1. The following are the IP addresses of the robot, the force-torque sensor, and the computer, which will all be important in this troubleshooting process (assuming that nobody has changed them from the time of the writing of this tutorial). They are 192.168.1.1, 192.168.1.2, and 192.168.1.50, respectively.

2. Try pinging the robot from the computer to make sure that they are connected. Open a command window on the computer, and then type "ping 192.168.1.1" and if there is a reply then the computer can communicate with the robot.
![image](https://user-images.githubusercontent.com/123105763/228034887-622b5fc4-77bd-4e40-805b-2728be8867e5.png)

3. Try pinging the force-torque sensor from the computer to make sure that they are connencted. In the command window, type "ping 192.168.1.2" and if there is a reply then the computer can communicate with the force-torque sensor.
![image](https://user-images.githubusercontent.com/123105763/228035224-6e75a495-a063-49b0-aeb3-5d924ba3428b.png)

4. You can type "ipconfig" into the command window to get the IPv4 address of the computer, 192.168.1.50, which should also appear on the pendant which tells you that the pendant is properly connected to the computer. To check, on the pendant navigate to menu > "Control Panel" > "Transmission Protocol" > "EGMSensor," and listed as the "Remote Address" should be the IPv4 address of the computer.
![image](https://user-images.githubusercontent.com/123105763/228036320-587ff076-a813-41e8-9ef5-852be94f13eb.png)

5. This completes troubleshooting by checking that all of the IP addresses are correct.
