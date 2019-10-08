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
```roslaunch mobile_manipulator_moveit_config demo.launch```
4. Motion planning using Rviz done

### Things to do

1. IK FAST plugin
2. Interfacing Dynamixel with moveIt 6dof
3. Generating python base codes from tutorial
4. getting started with c++ code for advanced usage
5. Gripper design(Hardware and solidworks)