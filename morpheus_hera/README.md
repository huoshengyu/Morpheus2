# Morpheus HERA

These instructions are for HERA Campaign 8 participants and assume that Morpheus has already been installed on an Ubuntu OS. Please see the main Morpheus readme if you need installation instructions or are using another OS.

All software and documentation herein is distributed under a GPLv3 License. See file "COPYING" for details.

As of 07/2025, the primary author and maintainer of this repository is Brian Sanyu Huo (bshuo@ucdavis.edu) at the Human/Robotics/Vehicle Integration & Performance Laboratory at the University of California, Davis (https://hrvip.ucdavis.edu/).

# Morpheus Hardware Launch Instructions

## 1. Power on the computer

<div class="alert alert-block alert-info">
<b>NOTE:</b> Turn on the computer first so that indicator lights on any connected devices will also power on. The computer should boot into Ubuntu. If it is a dual boot and boots into the wrong OS, hold F12 on startup to access the boot menu.
</div>

## 2. Power on the controller  

IRSS (miniature robot) controller:  
1. Plug power cable into a power outlet and the power supply board (larger board).
2. Plug USB cable into the computer's USB port and the U2D2 board (smaller plastic-cased board).
3. Flip the power supply board's power switch ON (dot side pressed down).
4. Verify that the red light on the power supply board is on.

Playstation 4 controller:  
1. Plug into computer's USB port.
2. Verify that the blue light on the front of the controller is on.

## 3. Power on the robot   

Trossen robot:  
1. Plug power cable into a power outlet and the power supply board (larger board).
2. Plug USB cable into the computer's USB port and the U2D2 board (smaller plastic-cased board).
3. Press the power button on the side of the robot.
4. Verify that the red light on the power supply board is on.

# Morpheus Software Launch Instructions (Ubuntu)

## 1. Open VSCode

You have two options to do this:
1. Click the VSCode icon in the Applications menu or Taskbar.
OR
2. Right click on the desktop. Select "New terminal" and enter `code`.

## 2. Start the Docker container

In VSCode, on the left sidebar:
1. Select the Containers sidebar tab (a box with vertical lines on its sides).
2. Under the "Containers" dropdown menu, right click on the container with "morpheus" in its name and select "Start".
3. Wait a moment and click the "Refresh" button until the red square becomes a green triangle.
4. Right click again on the same container and select "Attach Visual Studio Code". Any instruction labeled "Inside the Docker container" is to be carried out in this newly opened window, which is distinguishable by the blue bar in the bottom left which shows the container name.

## 3. Identify the correct launch filename:

Inside the Docker container: 
1. Select the "Explorer" sidebar tab and check the src/morpheus_hera/launch folder for a list of valid launch files.
2. Combine the following to find the corresponding `<filename>`:  
    |   |   |
    |---|---|
    | task type | (c=collision, t=trajectory) |
    | controller type | (p=ps4, g=gello) |
    | haptics condition | (y=yes haptics, n=no haptics) |  
    | task difficulty | (e=easy, m=medium, h=hard) |
For example, for a trajectory task, ps4 controller, with haptic feedback, and easy task, the correct `<filename>` is `tpye.launch`.  
Note that specifying the task, besides determining trajectory feedback, helps ensure your data recordings are properly categorized later.

## 4. Launch Morpheus ROS:

Inside the Docker container:
1. Using the launch file name found in the previous step, enter the launch command:  
`roslaunch morpheus_hera <filename>`  
For example, for trajectory guidance, ps4 controller, with haptic feedback, and easy task, the correct launch command is:  
`roslaunch morpheus_hera tpye.launch`  
Be sure to include the entire filename, including the underscores and the `.launch` suffix.
2. Visually verify that the robot activates and responds to controller inputs.
3. Visually verify that an RVIZ window opens and visualizes the robot, obstacle course, and either collision or trajectory arrows.

## 5. Record and complete a task trial:

Inside the Docker container:
1. In the terminal, follow the displayed instructions to input your subject ID, task name, and trial number in order to prepare for data recording.
2. Visually verify that the robot lifts slightly from the off position to the sleep position.
3. In the terminal follow the displayed instructions and press ENTER to begin recording a trial.
4. Use the controller to complete the task.
5. After completing the task, press ENTER or ESC to stop recording and end the trial.
6. After each trial, use the Explorer menu in VSCode to view the morpheus_data\data folder  and verify that the new data file has been created. You may need to press the "Refresh Explorer" button at the top of the Explorer menu in order to see the new file.

# Morpheus Software Shutdown Instructions

## 1. Shutdown ROS

Inside the Docker container:
1. In the same terminal in which you launched Morpheus, press ESC to exit data recording, then Ctrl+C to shut down ROS and the rest of the program.
2. Verify that the RVIZ window has closed and that any connected hardware (e.g. haptic sleeve, Gello controller) are not actuating.
3. At this point, you may close the VSCode window which is connected to the Docker container, as distinguished by the blue bar in the bottom left corner showing the container name.

## 2. Stop the Docker container:

Outside the Docker container:
1. On the left sidebar, return to the Containers menu.
2. Right click on the container with "morpheus" in its name, and select "Stop".
3. At this point, you may close the remaining VSCode window and begin hardware shutdown.

# Morpheus Hardware Shutdown Instructions

## 1. Power down controllers

IRSS (miniature robot) controller:  
1. No shutdown required, unless the controller needs to be stowed, relocated, or disassembled.
2. Flip the power supply board's power switch OFF (no-dot side pressed down).
3. Cables may be left plugged in or unplugged. The red light on the U2D2 board will remain on whenever the computer is connected and powered on.

Playstation 4 controller:  
1. No shutdown required. Controller goes to sleep automatically.

## 2. Power down laptop

1. Navigate to the taskbar on the top right of the Ubuntu desktop screen.
2. Click on "Power Off/Log Out" and select "Power Off...".
3. If prompted further, select "Power Off" to shutdown immediately.
4. Close the laptop when not in use to minimize wear and tear.

# Troubleshooting

## Can't find files/roslaunch file not found

Make sure your VSCode window and terminal are both in the correct directory, and that you are running any roslaunch commands inside the Docker container. In the main VSCode window, select the Explorer tab on the left sidebar (two pages icon) and check the folder name in the heading. In the terminal, check the directory shown at the beginning of any command. 

Outside the Docker container, the correct directory is:  
#TODO Check this on Linux laptop `/Morpheus Git`
Inside the Docker container, the correct directory is:  
`~/catkin_ws`

## No window opens on roslaunch/QT error/failed to connect to display  

Outside the Docker container, enter:  
`xhost +`  
then try again. This command permits programs in the Docker container to access the display.

If the issue persists, check that the DISPLAY environment variable is set correctly inside the Docker container. Outside the Docker container, enter "w" to list active displays. Inside the Docker container, enter:  #TODO Check these commands
`export DISPLAY=<display>`  
where `<display>` is the first listed display, usually `:0` or `:1`.