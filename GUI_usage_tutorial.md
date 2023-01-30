# Parts of the GUI
* World frame position and force readout
* Workpiece frame position and force readout
* Toolpath and logging
* Start/stop and mode selection
* Mode / status
* Toolpath execution
* Offsets
* Jog Control
* Safety checks and input validation

# Running a roll
1. Open RobotStudio and open the file that you created in setting up RobotStudio:  6640_controller_6_13_0_0.

2. Navigate to the dowloaded repository/folder for the GUI and right click > click "Edit" to edit the file deep_rolling_GUI.bat.
![image](https://user-images.githubusercontent.com/123105763/215500626-5f8e41d5-bc83-472c-840f-b2d54af43e6b.png)

3. Add the following two lines to the top of the file (everything inside of each set of quotation marks is one line):  "set QT_FONT_DPI=96" and "set QT_SCALE_FACTOR=1.5." Save the file.
![image](https://user-images.githubusercontent.com/123105763/215501014-c36e3327-4ea1-4a7c-8eac-349459e6b699.png)

4. Back in the downloaded folder, now double-click on deep_rolling_GUI.bat in order to run the batch file and spawn the GUI. 

5. Press the "Play" button in RobotStudio to start a simulation. Test that you can use the GUI to jog around the robot by translating it in all three directions. You may need to press "Stop" and then "Play" again in order to change the Status in the GUI to "Running." Jogging will not work until this status is set to "Running."
![image](https://user-images.githubusercontent.com/123105763/215501780-0dd91de9-2fe0-4859-88e4-1c3c01cf00c9.png)

6. In the GUI, change the mode to "Simulator" in the circular menu towards the bottom left.
![image](https://user-images.githubusercontent.com/123105763/215503276-f75f7b5f-3c0c-4dba-a4cc-414733ae3a8b.png)

7. Notice that most of the boxes in the bottom right of the GUI are red. The next objective is to turn all of them green, except for those having to do with the force/torque sensor which is not used in the simulation.
![image](https://user-images.githubusercontent.com/123105763/215503608-b31163e7-63a1-4f2f-ab84-b176f7d7048e.png)

8. In the GUI, press "Load config," and select the file in your repository "config" > "default_config.yaml." Four of the boxes in the lower right should turn green, as pictured in the image below (which also shows where the "Load config" button is).
![image](https://user-images.githubusercontent.com/123105763/215504018-798e754a-1306-498f-9632-e723d84682db.png)

9. In the GUI, press "Load toolpath," and select the file in your repository "src" > "toolpath_gen" > "short_toolpath.txt." One more box in the lower right should turn green, as pictured in the image below (which also shows where the "Load toolpath" button is).
![image](https://user-images.githubusercontent.com/123105763/215504223-5113ca84-d04c-469f-9c8e-c8dd992835df.png)

10. The status in the GUI should now say "Standby - Ready!" This is how you know that the simulation is ready to begin.
![image](https://user-images.githubusercontent.com/123105763/215504373-3d950bd6-9d80-4658-8881-a1919e6db5fe.png)

11. We would like to be able to see traces of the rolls that the robot makes in the simulation. To do this, go to the "Simulation" tab in RobotStudio, and then select "TCP Trace." Check off the box at the top of that menu that says "Enable TCP Trace." Also change the primary color of the trace to red so that it is easy to see.

![image](https://user-images.githubusercontent.com/123105763/215094928-01ec0aae-dd66-4e07-8769-37da02db9067.png)

12. You are now ready to begin the simulation. In the GUI, press the green "Start" button. The robot should begin to move and follow the toolpath. This simulation will be short, and by the end of it you should see a trace that looks something like the image below.
![image](https://user-images.githubusercontent.com/123105763/215095385-b7f6301a-d348-4ba8-a4a5-d6b438cd0e50.png)
