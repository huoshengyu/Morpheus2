# Follow docker install instructions in "Morpheus ROS Help" here:

[Morpheus Student Group Notion](https://www.notion.so/Morpheus-Student-Group-4d9b4c5947c34823b4dcc76b30b47f8b)



# In same directory as dockerfile:

Build the docker image and container on first run, or just open it on subsequent runs:
```
docker compose up
```

# In the VSCode Docker extension:

Right click on the morpheus docker container and select "Start", then "Attach Visual Studio Code".

# Inside the docker container:

Build catkin workspace (must do on first run and whenever C code is changed):
```
catkin build -s
```
Open a_bot in simulation (with keyboard controls: wasd, q/e, ijkl, u/o. NOTE: Must first press esc, then ctrl+c to cancel this command.): 
```
roslaunch morpheus_teleop a_bot_fake_keyboard_control.launch
```
Start the collision tracking node (and display nearest collisions in Rviz):
```
roslaunch morpheus_collision collision.launch
```
Start the data recording node:
```
roslaunch morpheus_data data.launch
```
Start the goal haptics node:
```
roslaunch morpheus_teleop goal_haptics.launch
```
