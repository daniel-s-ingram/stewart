Because the Stewart platform is a closed loop manipulator, the description was written in SDF rather than URDF. However, ROS does not support SDF by default, so a plugin was written to make the joints in Gazebo visible to ROS.


https://drive.google.com/file/d/1fv_EUJgSltfL8iSES1pD4P4oTfRB8rsK/view?usp=sharing


Clone this repo to your catkin workspace src directory and build it:


cd ~/your_catkin_ws_src_path/  
git clone https://github.com/daniel-s-ingram/stewart_ros.git  
catkin build  


To build the plugin:


cd plugin  
mkdir build  
cd build  
cmake ../  
make  


To set the gazebo_plugin_path variable, do the following:


echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/your_catkin_ws_src_path/stewart/plugin/build" >> ~/.bashrc  
source ~/.bashrc  


I plan to make the last step part of package.xml so it doesn't have to be done manually.
