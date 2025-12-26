# Morpheus

This repository combines UR and Trossen robot drivers, multiple hand controller types, robot and environment simulation, tools for collision and trajectory guidance and visualization, and data collection. It is currently in active development, and not all commits will be stable.

All software and documentation herein is distributed under a GPLv3 License. See file "COPYING" for details.

As of 07/2025, the primary author and maintainer of this repository is Brian Sanyu Huo (bshuo@ucdavis.edu) at the Human/Robotics/Vehicle Integration & Performance Laboratory at the University of California, Davis (https://hrvip.ucdavis.edu/).

# Morpheus Install Instructions

## 1. Install Docker

### Windows: Install Docker Desktop

Visit the Docker website and download the installer:  
https://www.docker.com/products/docker-desktop/

### Linux: Install Docker Engine via apt

Visit the Docker website and follow all instructions:
https://docs.docker.com/engine/install/ubuntu/

## 2. Install Docker extension in VScode

If not already installed, download VSCode:  
https://code.visualstudio.com/download

1. Open VScode. You can do this quickly by right-clicking on the desktop, opening a terminal, and entering `code`.  
2. On the left sidebar, find and select the Extensions tab.  
3. Search for and install the following extensions:  
    1. Docker
    2. Remote Development.

## 3. Clone Morpheus Git repository

In a terminal, navigate to an appropriate directory.  
1. Run the following to clone Morpheus:  
```
mkdir morpheus_git
cd morpheus_git
git clone https://github.com/huoshengyu/Morpheus  
```  
2. Clone and update all submodules:
```
git submodule update --init --recursive --checkout --progress
```

## 4. Build the Docker image:

In a terminal, check that the current directory is morpheus_git.  
1. Build the Docker image:
```
docker compose build
```
2. Check that the build completes without errors (red messages). Warnings (yellow messages) are acceptable.
You do not need to rebuild the docker image unless you make changes to the Dockerfile.

## 5. Start the Docker container:

To start the docker container on first run:
1. Run the following:
```
docker compose up
```
On future runs, repeat the above, or:  
1. In VScode, on the left sidebar, find and select the Docker tab.  
2. Right click on the Morpheus Docker container and select "Start".

## 6. Open a VSCode window in the Docker container:

In VScode, on the left sidebar:  
1. Find and select the Docker tab.  
2. Right click on the morpheus docker container and select "Attach Visual Studio Code".

<div class="alert alert-block alert-info">
<b>NOTE:</b> You should now have two VSCode windows: One outside the Docker container and one inside the Docker container. The one inside the Docker container will have a blue bar on the bottom left showing the container name.
</div>

## 7. Check display settings:

To allow RVIZ and/or Gazebo to open:  
1. Outside the Docker container, allow connection to the display:  
```
xhost +
```
2. You may need to manually set the `DISPLAY` environment variable to one of the active displays if it did not set correctly.
    1. Outside the Docker container, list active displays:  
        ```
        w
        ```

    2. Inside the Docker container, check what `DISPLAY` is set to:  
        ```
        echo $DISPLAY
        ```
    3. Set `DISPLAY` to match a result from `w`. For example, if `w` shows that display `:1` exists:  
        ```
        export DISPLAY=:1
        ```

## 8. Build:

1. Open a new terminal (In VSCode, on the top bar, click "Terminal" and select "New Terminal").  
2. Build catkin workspace in the new terminal (must do on first run and whenever C code is changed):
```
catkin build
```

# Morpheus Hardware Launch Instructions

## 1. Power on the computer

<div class="alert alert-block alert-info">
<b>NOTE:</b> Turn on the computer first so that indicator lights on any connected devices will also power on. The computer should boot into Ubuntu. If it is a dual boot and boots into the wrong OS, hold F12 on startup to access the boot menu.
</div>

## 2. Power on the controller  

IRSS (miniature robot) controller:  
1. Plug power cable into a power outlet and the power supply board (larger board)
2. Plug USB cable into the computer's USB port and the U2D2 board (smaller plastic-cased board)
3. Flip the power supply board's power switch ON (dot side pressed down)
4. Verify that the red light on the power supply board is on.

Playstation 4 controller:  
1. Plug into computer's USB port.  
2. Verify that the blue light on the front of the controller is on.

## 3. Power on the robot   

UR robot:  
1. Press the power button on the Universal Robots Teach Pendant (the small monitor attached to the UR's control box). Wait for the teach pendant to start up.   
2. Open the boot menu (red dot in the bottom left corner) and start the robot itself.

Trossen robot:  
1. Plug power cable into a power outlet and the power supply board (larger board)
2. Plug USB cable into the computer's USB port and the U2D2 board (smaller plastic-cased board)
3. Press the power button on the side of the robot
4. Verify that the red light on the power supply board is on.

## 4. Power on the gripper 

OnRobot RG2-FT gripper:  
1. Plug power cable into power outlet
2. Plug green control cable into OnRobot gripper
3. Plug ethernet cable into ethernet adapter (which is also plugged in to the computer and robot control box)
4. Verify that light on side of gripper turns on.

Robotiq 2f-85 gripper:
1. Powered automatically by the robot.
2. Verify that light on side of gripper turns on.

<div class="alert alert-block alert-info">
<b>NOTE:</b> You may need to run ```gello_software/tool_communication.py``` to use the Robotiq gripper with the IRSS (but this should be run automatically by the launch files).
</div>

## 5. UR5e: Set the robot control mode

If using IRSS controller:  
1. Set the robot to remote control using the Teach Pendant. The button is near the top right of the Teach Pendant's screen.

If using Xbox/Playstation controller:  
1. Set the robot to local control using the Teach Pendant. The button is near the top right of the Teach Pendant's screen.
2. On the Programs tab, load and start ros_control. You may need to perform this step after the Morpheus bringup launch file is running.

# Morpheus Software Launch Instructions

## 1. Windows: Launch Docker

Open Docker Desktop and allow ~1 minute to start up.

This step is not necessary on Linux.

## 2. Open VSCode

Open a new terminal and enter `code`.

## 3. Start the Docker container

In VSCode, on the left sidebar:
1. Select the Docker extension tab
2. Right click on the Morpheus Docker container and select "Attach Visual Studio Code"

## 4. Launch Morpheus:

### All at once:

Inside the Docker container:  
```
roslaunch morpheus_main a_bot_main.launch
```
<div class="alert alert-block alert-info">
<b>NOTE:</b> As of 2/2025, the ...main.launch files do not launch the spawner node (to avoid race condition with the collision node) or the data collection node (to avoid unwanted file creation).
</div>

### Part by part:

Inside the Docker container:  
1. Open a_bot in simulation with keyboard controls:


    ```
    roslaunch morpheus_teleop a_bot_fake_keyboard_control.launch
    ```
    <div class="alert alert-block alert-info">
    <b>NOTE:</b> To cancel/end the above command, you must first press esc, then ctrl+c. Keyboard controls use the following keys: qweasd, uiojkl.
    </div>

3. Open a new terminal. Start the collision tracking node (and display nearest collisions in Rviz):
```
roslaunch morpheus_collision collision.launch
```
3. Open a new terminal. Start the data recording node:
```
roslaunch morpheus_data data.launch
```
4. Open a new terminal. Start the goal haptics node:
```
roslaunch morpheus_teleop goal_haptics.launch
```

## 5. Launch the IRSS

Inside the Docker container:  
1. Check that the robot is near the home position and not near any possibe collisions. Launching the IRSS will cause the robot to move to the home position.
2. Open a new terminal. Start the IRSS node:
```
./gello_software/run_gello_onrobot.sh
```
3. Check the joint angles. The IRSS will not start unless the joint angles of the controller also match the home position.
4. Repeat steps 2-3 until the IRSS is correctly launched.

# Morpheus Software Arduino Side
Before running the experiment, provide one of two sleeves (collision avoidance = long sleeve, trajectory guidance = short sleeve)
Set up the Arduino serial connectivity:
1. Ctrl + T (open terminal on the UBUNTU)
2. Type
```
ls -l /dev/arduino
```
3. Expected output:
```
/dev/arduino -> ttyUSB1 or ttyUSB0
```
Optional 4. If output does not look like the above:
```
sudo nano /etc/udev/rules.d/99-arduino.rules
```
Optional 5. Copy and paste it, then save it (Ctrl+S -> Ctrl+X)
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino"
```
5. If output does look like the above: (you can skip Optional 4 & Optional 5) Type the below
```
sudo udevadm control --reload-rules
```
```
sudo udevadm trigger
```

# UC Davis experimental settings

## 1. Ubuntu: Open the terminal
Open terminal first with a short key: "CTRL+ALT+T"

## 2. On the Ubutu terminal, check the connectivity of eye-tracking:
Open Pupil Neon Companion App: Click the "Top-Right corner icon" on the main page.
You can find the IP-Address. Then,
```
ping <IP-Address>
```
Then, you can see similar output (e.g., "64 bytes from 192.168.0.102: icmp_seq=1 ttl=64 time=369 ms")
```
sudo tcpdump -i wlo1 udp port 16571
```
Test whether the Pupil headset (Neon or eye-tracking) is sending broadcast / discovery packets on the network.

## 3. Allow connection to the display
1. Outside the Docker container, allow connection to the display:  
```
xhost +
```
2. You may need to manually set the `DISPLAY` environment variable to one of the active displays if it did not set correctly.
    1. Outside the Docker container, list active displays:  
        ```
        w
        ```
    2. Inside the Docker container, check what `DISPLAY` is set to:  
        ```
        echo $DISPLAY
        ```
    3. Set `DISPLAY` to match a result from `w`. For example, if `w` shows that display `:1` exists:  
        ```
        export DISPLAY=:1
        ```
## 4. Open VS code on the Ubuntu terminal
```
code
```
## 5. Start the Docker container
Before, check the path directory: "bear@BEAR:~/morpheus_git/Morpheus"
If yes, 
```
docker compose up
```
In VSCode, on the left sidebar:
1. Select the Docker extension tab
2. Right click on the Morpheus Docker container and select "Attach Visual Studio Code"

## 6. Launch Morpheus:
```
roslaunch morpheus_hera cpne.launch
```


# Additional Installation Notes

Morpheus Student Group Notion:  
https://www.notion.so/Morpheus-Student-Group-4d9b4c5947c34823b4dcc76b30b47f8b
