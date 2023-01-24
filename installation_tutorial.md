# Install Python 3
1. Go to python.org. Click on the "Downloads" tab near the top, and then click to download the proper version of Python for your computer.
![image](https://user-images.githubusercontent.com/123105763/214297751-7b07b5f2-0ebe-4135-9283-9808964e802a.png)

2. click on the installer file which was downloaded on your computer. Click "Modify" > "Next" and then check the box "Add Python to environment variables."
![image](https://user-images.githubusercontent.com/123105763/214298020-5f3071a5-e429-47a0-a32b-3f6de4daf6ad.png)

3. Finish the installation by clicking "Install."

4. Restart your computer before proceeding.

# Install dependencies
1. Open the Command Prompt on your computer by typing "Command Prompt" into the search bar.

2. Type the following command (everything inside of the quotation marks) into the Command Prompt and press enter:  "python -m pip install general-robotics-toolbox numpy pyqt5 protobuf==3.20.3 beautifulsoup4 ws4py pyyaml pandas openpyxl requests"

# Run GUI
1. Go to https://github.com/aelias36/robotic-deep-rolling-gui, click on the green "Code" tab near the top right, and then click "Download ZIP."
![image](https://user-images.githubusercontent.com/123105763/214307550-901beff8-fdfb-497f-bd10-151f03211123.png)

2. Once the ZIP file downloads, right click on it in your file explorer, and then select "Extract All" to unzip the files.

3. Within the folder, right click on the file called "deep_rolling_GUI.bat." Open the file in Notepad. Change "python3" to "python" in this file, and then save it.
![image](https://user-images.githubusercontent.com/123105763/214308403-f58338e6-0aba-4885-b0a6-0701771cec03.png)

4. After the batch file saves, go back to the file explorer. Double click on the batch file to run it.

5. The GUI should automatically open after the batch file runs.
![image](https://user-images.githubusercontent.com/123105763/214309294-211363e3-38e6-4044-aa94-b7a081ab1726.png)
