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

2. Navigate to the dowloaded repository/folder for the GUI and run the batch file (called deep_rolling_GUI.bat) in order to open the GUI.
![image](https://user-images.githubusercontent.com/123105763/215092083-c15f8cb0-2059-409d-a2f9-1d39454bcd27.png)

3. Press the "Play" button in RobotStudio to start a simulation. Test that you can use the GUI to jog around the robot by translating it in all three directions. You may need to press "Stop" and then "Play" again in order to change the Status in the GUI to "Running." Jogging will not work until this status is set to "Running."
![image](https://user-images.githubusercontent.com/123105763/215092828-961cf096-1101-430d-90c8-bdd6a17abf00.png)

4. In the GUI, change the mode to "Simulator" in the circular menu towards the bottom left.
![image](https://user-images.githubusercontent.com/123105763/215093002-fc0ab133-113a-4ff6-9a02-c52703ef7073.png)

5. Notice that most of the boxes in the bottom right of the GUI are red. The next objective is to turn all of them green, except for those having to do with the force/torque sensor which is not used in the simulation.
![image](https://user-images.githubusercontent.com/123105763/215093170-9474eb85-c8b0-444b-845b-e65392f93482.png)

6. In the GUI, press "Load config," and select the file in your repository "config" > "default_config.yaml." Four of the boxes in the lower right should turn green, as pictured in the image below (which also shows where the "Load config" button is).
![image](https://user-images.githubusercontent.com/123105763/215093865-c1563d00-67b9-4d33-9fbf-d4531b033704.png)

7. In the GUI, press "Load toolpath," and select the file in your repository "src" > "toolpath_gen" > "short_toolpath.txt." One more box in the lower right should turn green, as pictured in the image below (which also shows where the "Load toolpath" button is).
![image](https://user-images.githubusercontent.com/123105763/215094262-4a9dd2b3-503f-423b-8304-b0673930c0fc.png)

8. The status in the GUI should now say "Standby - Ready!" This is how you know that the simulation is ready to begin.
![image](https://user-images.githubusercontent.com/123105763/215094532-889c440c-9177-48dd-851e-10d60696fb6c.png)

9. We would like to be able to see traces of the rolls that the robot makes in the simulation. To do this, go to the "Simulation" tab in RobotStudio, and then select "TCP Trace." Check off the box at the top of that menu that says "Enable TCP Trace." Also change the primary color of the trace to red so that it is easy to see.

![image](https://user-images.githubusercontent.com/123105763/215094928-01ec0aae-dd66-4e07-8769-37da02db9067.png)

10. You are now ready to begin the simulation. In the GUI, press the green "Start" button. The robot should begin to move and follow the toolpath. This simulation will be short, and by the end of it you should see a trace that looks something like the image below.
![image](https://user-images.githubusercontent.com/123105763/215095385-b7f6301a-d348-4ba8-a4a5-d6b438cd0e50.png)
