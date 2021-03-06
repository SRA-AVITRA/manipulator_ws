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
8. IK FAST plugin
9. Generating python base codes from tutorial

### Things to do

1. getting started with c++ code for advanced usage
2. Gripper design(Hardware and solidworks)
3. Detect grasp orientation for detected objects

## Installations 

1. Manipulator
```
sudo apt-get install ros-kinetic-dynamixel-controllers
sudo apt install ros-kinetic-moveit
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
sudo apt-get install ros-kinetic-timed-roslaunch
```
2. Perception
```
sudo apt-get install ros-kinetic-realsense2*
sudo apt-get insatall ros-kinetic-librealsense2
sudo apt-get insatall ros-kinetic-librealsense2-dev
sudo apt-get insatall ros-kinetic-librealsense2-dkms
sudo apt-get insatall ros-kinetic-librealsense2-utils
```

## For manipulator 
1. Start manipulator
  
   _For simulation arm_ : ```roslaunch mobile_manipulator_moveit_config demo.launch``` 
    
   _For hardware arm_ :```roslaunch my_dynamixel_tutorial execute.launch``` 	  
   _For rviz with hardware_ : ```roslaunch my_dynamixel_tutorial rviz.launch```		 

2.  Run python scripts from move_group

## Perception Pipeline 
1. Start the RealSense Node
    
    ```roslaunch realsense2_camera rs_rgbd.launch```
    
2. Start the pixel centroid and physical centroid nodes

    ```rosrun perception pixel_centroid.py```
    ```rosrun perception position_3d.py```
 
3. Segmentation 
 
    ```rosrun my_pcl_tutorial segmentation```

