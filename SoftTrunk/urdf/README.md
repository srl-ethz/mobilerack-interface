Holds URDF files that describe the robot.
Uses [XACRO](http://wiki.ros.org/xacro) for macros and such to make the URDF simpler.

To generate a URDF file from XACRO file, run `./create_urdf.sh`.

View the model in Rviz with `roslaunch rviz.launch`.

The URDF model currently does not accurately reflect the actual augmented model- needs two more joints in the center.