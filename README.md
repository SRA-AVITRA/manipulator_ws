# 6DOF Manipulator

This is the clean version of 6DOF Manipulator 2019 code. We want to keep it this way.
Rules

Add documentation as you go Every commit should be named well, but far more importantly, add any functionality (and where to find it, and how to run it) in the documentation section below.

Add any testing files to a gitignore unless they will be helpful for everyone else Ultimately, we want this repo to be as clean as any open sourced library, so let's only keep code that will actually be used this year in some form.

Develop on the dev branch Obviously, we'll want to keep a development branch with tests, random stuff, etc.; keep this on the dev branch, and only merge actual functions into master.
Documentation

Soon to come! Let's get everything working first. I realize this is against rule 1; it will be fixed soon.


## Work done
 
1. 6dof manipulator urdf added
2. Removed squarePlate because .dae was crashing
3. MoveIt congfig generated
4. Motion planning using Rviz done
5. TRAC-IK solver tested
6. Perception using kinect started
7. Gripper started

### Things to do

1. IK FAST plugin
2. Generating python base codes from tutorial
3. getting started with c++ code for advanced usage
4. Gripper design(Hardware and solidworks)

## Installations 
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin

## For manipulator 
1. Start manipulator
  
   _For simulation arm_ : ```roslaunch mobile_manipulator_moveit_config demo.launch``` 
    
   _For hardware arm_ :```roslaunch my_dynamixel_tutorial execute.launch``` 	  
   _For rviz with hardware_ : ```roslaunch my_dynamixel_tutorial rviz.launch```		 

2.  Run python scripts from move_group

### For Perception Pipeline

    roslaunch openni_launch openni.launch
    rosrun perception pixel_xy.py
    rosrun perception depth_from_pixels.py
    To extract colour from object, run "rosrun perception get_colors.py" and select a bounding box of the relevant objects. This will print the hsv values of the object. Enter these values in the pixel_xy.py script in place of lowerRed and upperRed values

### To install openni and associated packages for perception

    sudo apt-get install ros-kinetic-openni-camera
    sudo apt-get install ros-kinetic-openni-launch
    sudo apt-get install ros-kinetic-ros-numpy
    sudo apt-get install ros-kinetic-cv-bridge
    sudo apt-get install ros-kinetic-vision-opencv

