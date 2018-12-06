![](https://github.com/daniel-s-ingram/stewart/gif/stewart.gif)

Because the Stewart platform is a closed loop manipulator, the description was written in SDF rather than URDF. However, ROS does not support SDF by default, so a plugin was written to make the joints in Gazebo visible to ROS.

Here is a link to an in-progress generalized version of this plugin: https://github.com/daniel-s-ingram/ros_sdf.

Clone this repo to your catkin workspace src directory and build it:

```
cd ~/your_catkin_ws_src_path/  
git clone https://github.com/daniel-s-ingram/stewart.git  
catkin build stewart
source ~/your_catkin_ws/devel/setup.bash
```

To build the plugin:

```
cd plugin  
mkdir build  
cd build  
cmake ../  
make  
```

Create the SDF file from the ERB template:

```
cd ../../sdf/stewart/
erb model.sdf.erb > model.sdf
```

Now, to launch the package:

```
roslaunch stewart stewart.launch
```



Note: The package currently expects a Dualshock 3 or 4 controller to be connected via USB.

The left analog stick translates in x and y. R2 increases z and L2 decreases z.
The right analog stick rotates around the x and y axes (roll and pitch). R1 and L1 should increase and decrease yaw, respectively, but this isn't yet working for some reason.x
