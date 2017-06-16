
@bennypi has indicated that he's no longer working on this project and encouraged me to take it over. 
I've made some fixes so that the code under src/ now works with C++11 and Gazebo 7.x . 
I'll push up further changes and add more features/functionality as time permits. 

Latest changes:
-   Bug fixes to code under src/ so it works with Gazebo 7.x
-   Re-exported/renamed .stl and .dae files to fix some geometry errors with the meshes and make the model overall easier to work with. 
-   Added RasPiCam model and Camera plugin to provide a camera onto the end of the arm
-   Added limits/constraints/other physics attributes to make the arm respond more realistically. 
-   Moved from shell scripts to proper ROS launch files for launching/startup. 
-   Organized repo into proper ROS REP-compliant directory structure. 
-   Adding more sensors
___________________

uarm-in-gazebo
==============
We are planning to create a working model of the uarm in gazebo. The uarm is a open-source robotic arm which was created by uFactory and founded through Kickstarter https://www.kickstarter.com/projects/ufactory/uarm-put-a-miniature-industrial-robot-arm-on-your.

Our plan (3. and 4. are bonuses if we have enough time):

1. Create a urdf-file from the CAD-model of the uarm and possibly merge or remove some parts to optimize it for the simulation. DONE
2. Create plugins to control the arm. DONE
3. Add a endeffector to the arm. DONE
4. Connect the arm to ROS.

Installation
==============
The following installation is tested against Gazebo 1.9.5.

1. Clone this repository.
2. Go into the directory "uarm-in-gazebo"
3. mkdir build && cd build
4. cmake ../src/
5. make
6. cd ..


Usage
==============
After the installation you can start the simulation with the following command:
```
launch_simulation.bash
```
or
```
launch_simulation.zsh
```

Now you can use `UarmPub` to send commands to the uarm. It's possible to send an angle in radian to a specific joint:
```
build/UarmPub joint_name angle
```
Controllable joints are *left_base_shoulder_joint*, *left_base_arm_joint* and *center_table_mount*.
Alternatively you can give the `UarmPub` a specific pose that the uarm should take:
```
build/UarmPub pose
```
At the moment the only available pose is *home_pose* which gives all three joints the angle 0.

Also you can start another world simulation where you can attach or dettach a small box with a suction cup.
Therefor you have to execute the following command:
```
launch_simulation_with_box.bash
```
or
```
launch_simulation_with_box.zsh
```
Now you have to navigate the suction cup over the box. You can use the following commands:
```
build/UarmPub "left_base_shoulder_joint" -0.6
```
and
```
build/UarmPub "left_base_arm_joint" -0.9
```
The suction cup should be over the box now. To attach the box to the suction cup, you can use the following command:
```
build/UarmPub "attach" 1
```
After you attached the box it is possible to move the arm like described above and the box should be at the suction cup all time.
If you want to dettach the box, you have to use the following command:
```
build/UarmPub "attach" 0
```
















