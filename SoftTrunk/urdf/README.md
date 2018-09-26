Holds URDF files that describe the robot.
Uses [XACRO](http://wiki.ros.org/xacro) for macros and such to make the URDF simpler.

To generate a URDF file from XACRO file, run `rosrun xacro xacro --inorder -o robot.urdf robot.urdf.xacro`.

View the model in Rviz with `roslaunch rviz.launch`.
