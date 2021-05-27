cd ./PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# launches gazebo_ros, mavros, sitl and px4 with custom vehicle, world and pose
# x:=-91.7 y:=4.56 z:=21
roslaunch px4 mavros_posix_sitl.launch vehicle:=typhoon_h480 world:=$(pwd)/../worlds/hills.world x:=-25.7 y:=27 z:=-1.2
# roslaunch px4 mavros_posix_sitl.launch world:=$(pwd)/../worlds/hills.world x:=-25.7 y:=27 z:=-1.2
