# PX4 Setup Guide

## Prerequisites
---
Setup is intended to run on a clean Ubuntu LTS 18.04 installation

## Install toolchain for PX4 development
---
1. Download PX4 source code with custom files by cloning the following git repository
```
cd ~
git clone https://github.com/dylantzx/PX4.git --recursive
```
2. Run shell script to install common dependencies and tools for Nuttx, jMAVSim and Gazebo
```
bash ./PX4/PX4-Autopilot/Tools/setup/ubuntu.sh
```
3. Restart computer upon completion
4. Re-run step 2 to check if everything has been installed.

## Install ROS Melodic, Gazebo9, PX4 and MAVROS and other necessary dependencies 
---
1. Download script in a bash shell
```
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
```
2. Run the script to install
```
bash ubuntu_sim_ros_melodic.sh
```
3. Re-run step 2 to check if everything has been installed.
4. Some of the MAVROS are not downloaded, so you have to run this extra command:
```
sudo apt-get install ros-melodic-mavros*
```
## QGroundControl 
---
When installing QGroundControl for the first time, you have to remove modem manager and grant permission to access the serial port. You also need to install GStreamer to allow video streaming.

**Note: The QGroundControl App Image has already been downloaded with permission given to run as an executable.** 

1. Run the following commands:
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
2. Re-login to enable change.

3. Install a few other necessary plugins
```
sudo apt install libgstreamer1.0-dev
sudo apt install gstreamer1.0-plugins-good
sudo apt install gstreamer1.0-plugins-bad
sudo apt install gstreamer1.0-plugins-ugly
```
## Error Debugging
---
You may face an installation error for Gazebo. To resolve error:
1. Change directory. If directory not found, run Gazebo in terminal first
```
cd ~/.ignition/fuel
```
2. Open config.yaml file 
```
gedit config.yaml
```
3. Replace `url: https://api.ignitionfuel.org` with `url: https://api.ignitionrobotics.org`.

## Final steps
---
Run these steps
```
sudo apt-get upgrade
sudo apt-get update
```
## How to run PX4 and Gazebo Simulation with MavROS
---

The `px4.sh` shell script handles the necessary commands to successfully launch PX4, Gazebo and MavROS. The script is currently running the `multi_uav_mavros_sitl.launch` launch file with settings `<arg name="gui" default="false"/>` in line 10.

If you want Gazebo to be launched, go to `~/PX4/PX4-Autopilot/launch/multi_uav_mavros_sitl.launch` and change the default settings to `true`.

To run the script:

1. Change into PX4 directory and run the shell script
```
cd ~/PX4
./px4.sh 
```
The above command runs the script in a child terminal. If you want to run the script in your current terminal, you can run
```
cd ~/PX4
source px4.sh
```
2. When you run `rostopic list` in a separate terminal, you should be able to see both gazebo and mavros topics
```
/clock
/diagnostics
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/rosout
/rosout_agg
/tf
/tf_static
/uav0/camera/camera_info
/uav0/camera/image_raw
/uav0/camera/image_raw/compressed
/uav0/camera/image_raw/compressed/parameter_descriptions
/uav0/camera/image_raw/compressed/parameter_updates
/uav0/camera/image_raw/compressedDepth
/uav0/camera/image_raw/compressedDepth/parameter_descriptions
/uav0/camera/image_raw/compressedDepth/parameter_updates
/uav0/camera/image_raw/theora
/uav0/camera/image_raw/theora/parameter_descriptions
/uav0/camera/image_raw/theora/parameter_updates
/uav0/camera/parameter_descriptions
/uav0/camera/parameter_updates
/uav0/mavlink/from
/uav0/mavlink/gcs_ip
/uav0/mavlink/to
/uav0/mavros/actuator_control
/uav0/mavros/adsb/send
/uav0/mavros/adsb/vehicle
/uav0/mavros/altitude
/uav0/mavros/battery
/uav0/mavros/cam_imu_sync/cam_imu_stamp
/uav0/mavros/companion_process/status
/uav0/mavros/debug_value/debug
/uav0/mavros/debug_value/debug_vector
/uav0/mavros/debug_value/named_value_float
/uav0/mavros/debug_value/named_value_int
/uav0/mavros/debug_value/send
/uav0/mavros/esc_info
/uav0/mavros/esc_status
/uav0/mavros/estimator_status
/uav0/mavros/extended_state
/uav0/mavros/fake_gps/mocap/tf
/uav0/mavros/geofence/waypoints
/uav0/mavros/global_position/compass_hdg
/uav0/mavros/global_position/global
/uav0/mavros/global_position/gp_lp_offset
/uav0/mavros/global_position/gp_origin
/uav0/mavros/global_position/home
/uav0/mavros/global_position/local
/uav0/mavros/global_position/raw/fix
/uav0/mavros/global_position/raw/gps_vel
/uav0/mavros/global_position/raw/satellites
/uav0/mavros/global_position/rel_alt
/uav0/mavros/global_position/set_gp_origin
/uav0/mavros/gps_rtk/rtk_baseline
/uav0/mavros/gps_rtk/send_rtcm
/uav0/mavros/gpsstatus/gps1/raw
/uav0/mavros/gpsstatus/gps1/rtk
/uav0/mavros/gpsstatus/gps2/raw
/uav0/mavros/gpsstatus/gps2/rtk
/uav0/mavros/hil/actuator_controls
/uav0/mavros/hil/controls
/uav0/mavros/hil/gps
/uav0/mavros/hil/imu_ned
/uav0/mavros/hil/optical_flow
/uav0/mavros/hil/rc_inputs
/uav0/mavros/hil/state
/uav0/mavros/home_position/home
/uav0/mavros/home_position/set
/uav0/mavros/imu/data
/uav0/mavros/imu/data_raw
/uav0/mavros/imu/diff_pressure
/uav0/mavros/imu/mag
/uav0/mavros/imu/static_pressure
/uav0/mavros/imu/temperature_baro
/uav0/mavros/imu/temperature_imu
/uav0/mavros/landing_target/lt_marker
/uav0/mavros/landing_target/pose
/uav0/mavros/landing_target/pose_in
/uav0/mavros/local_position/accel
/uav0/mavros/local_position/odom
/uav0/mavros/local_position/pose
/uav0/mavros/local_position/pose_cov
/uav0/mavros/local_position/velocity_body
/uav0/mavros/local_position/velocity_body_cov
/uav0/mavros/local_position/velocity_local
/uav0/mavros/log_transfer/raw/log_data
/uav0/mavros/log_transfer/raw/log_entry
/uav0/mavros/manual_control/control
/uav0/mavros/manual_control/send
/uav0/mavros/mission/reached
/uav0/mavros/mission/waypoints
/uav0/mavros/mocap/pose
/uav0/mavros/mount_control/command
/uav0/mavros/mount_control/orientation
/uav0/mavros/obstacle/send
/uav0/mavros/odometry/in
/uav0/mavros/odometry/out
/uav0/mavros/onboard_computer/status
/uav0/mavros/param/param_value
/uav0/mavros/play_tune
/uav0/mavros/px4flow/ground_distance
/uav0/mavros/px4flow/raw/optical_flow_rad
/uav0/mavros/px4flow/raw/send
/uav0/mavros/px4flow/temperature
/uav0/mavros/radio_status
/uav0/mavros/rallypoint/waypoints
/uav0/mavros/rc/in
/uav0/mavros/rc/out
/uav0/mavros/rc/override
/uav0/mavros/setpoint_accel/accel
/uav0/mavros/setpoint_attitude/cmd_vel
/uav0/mavros/setpoint_attitude/thrust
/uav0/mavros/setpoint_position/global
/uav0/mavros/setpoint_position/global_to_local
/uav0/mavros/setpoint_position/local
/uav0/mavros/setpoint_raw/attitude
/uav0/mavros/setpoint_raw/global
/uav0/mavros/setpoint_raw/local
/uav0/mavros/setpoint_raw/target_attitude
/uav0/mavros/setpoint_raw/target_global
/uav0/mavros/setpoint_raw/target_local
/uav0/mavros/setpoint_trajectory/desired
/uav0/mavros/setpoint_trajectory/local
/uav0/mavros/setpoint_velocity/cmd_vel
/uav0/mavros/setpoint_velocity/cmd_vel_unstamped
/uav0/mavros/state
/uav0/mavros/statustext/recv
/uav0/mavros/statustext/send
/uav0/mavros/target_actuator_control
/uav0/mavros/time_reference
/uav0/mavros/timesync_status
/uav0/mavros/trajectory/desired
/uav0/mavros/trajectory/generated
/uav0/mavros/trajectory/path
/uav0/mavros/vfr_hud
/uav0/mavros/vision_pose/pose
/uav0/mavros/vision_pose/pose_cov
/uav0/mavros/vision_speed/speed_twist_cov
/uav0/mavros/wind_estimation
/uav1/mavlink/from
/uav1/mavlink/gcs_ip
/uav1/mavlink/to
/uav1/mavros/actuator_control
/uav1/mavros/adsb/send
/uav1/mavros/adsb/vehicle
/uav1/mavros/altitude
/uav1/mavros/battery
/uav1/mavros/cam_imu_sync/cam_imu_stamp
/uav1/mavros/companion_process/status
/uav1/mavros/debug_value/debug
/uav1/mavros/debug_value/debug_vector
/uav1/mavros/debug_value/named_value_float
/uav1/mavros/debug_value/named_value_int
/uav1/mavros/debug_value/send
/uav1/mavros/esc_info
/uav1/mavros/esc_status
/uav1/mavros/estimator_status
/uav1/mavros/extended_state
/uav1/mavros/fake_gps/mocap/tf
/uav1/mavros/geofence/waypoints
/uav1/mavros/global_position/compass_hdg
/uav1/mavros/global_position/global
/uav1/mavros/global_position/gp_lp_offset
/uav1/mavros/global_position/gp_origin
/uav1/mavros/global_position/home
/uav1/mavros/global_position/local
/uav1/mavros/global_position/raw/fix
/uav1/mavros/global_position/raw/gps_vel
/uav1/mavros/global_position/raw/satellites
/uav1/mavros/global_position/rel_alt
/uav1/mavros/global_position/set_gp_origin
/uav1/mavros/gps_rtk/rtk_baseline
/uav1/mavros/gps_rtk/send_rtcm
/uav1/mavros/gpsstatus/gps1/raw
/uav1/mavros/gpsstatus/gps1/rtk
/uav1/mavros/gpsstatus/gps2/raw
/uav1/mavros/gpsstatus/gps2/rtk
/uav1/mavros/hil/actuator_controls
/uav1/mavros/hil/controls
/uav1/mavros/hil/gps
/uav1/mavros/hil/imu_ned
/uav1/mavros/hil/optical_flow
/uav1/mavros/hil/rc_inputs
/uav1/mavros/hil/state
/uav1/mavros/home_position/home
/uav1/mavros/home_position/set
/uav1/mavros/imu/data
/uav1/mavros/imu/data_raw
/uav1/mavros/imu/diff_pressure
/uav1/mavros/imu/mag
/uav1/mavros/imu/static_pressure
/uav1/mavros/imu/temperature_baro
/uav1/mavros/imu/temperature_imu
/uav1/mavros/landing_target/lt_marker
/uav1/mavros/landing_target/pose
/uav1/mavros/landing_target/pose_in
/uav1/mavros/local_position/accel
/uav1/mavros/local_position/odom
/uav1/mavros/local_position/pose
/uav1/mavros/local_position/pose_cov
/uav1/mavros/local_position/velocity_body
/uav1/mavros/local_position/velocity_body_cov
/uav1/mavros/local_position/velocity_local
/uav1/mavros/log_transfer/raw/log_data
/uav1/mavros/log_transfer/raw/log_entry
/uav1/mavros/manual_control/control
/uav1/mavros/manual_control/send
/uav1/mavros/mission/reached
/uav1/mavros/mission/waypoints
/uav1/mavros/mocap/pose
/uav1/mavros/mount_control/command
/uav1/mavros/mount_control/orientation
/uav1/mavros/obstacle/send
/uav1/mavros/odometry/in
/uav1/mavros/odometry/out
/uav1/mavros/onboard_computer/status
/uav1/mavros/param/param_value
/uav1/mavros/play_tune
/uav1/mavros/px4flow/ground_distance
/uav1/mavros/px4flow/raw/optical_flow_rad
/uav1/mavros/px4flow/raw/send
/uav1/mavros/px4flow/temperature
/uav1/mavros/radio_status
/uav1/mavros/rallypoint/waypoints
/uav1/mavros/rc/in
/uav1/mavros/rc/out
/uav1/mavros/rc/override
/uav1/mavros/setpoint_accel/accel
/uav1/mavros/setpoint_attitude/cmd_vel
/uav1/mavros/setpoint_attitude/thrust
/uav1/mavros/setpoint_position/global
/uav1/mavros/setpoint_position/global_to_local
/uav1/mavros/setpoint_position/local
/uav1/mavros/setpoint_raw/attitude
/uav1/mavros/setpoint_raw/global
/uav1/mavros/setpoint_raw/local
/uav1/mavros/setpoint_raw/target_attitude
/uav1/mavros/setpoint_raw/target_global
/uav1/mavros/setpoint_raw/target_local
/uav1/mavros/setpoint_trajectory/desired
/uav1/mavros/setpoint_trajectory/local
/uav1/mavros/setpoint_velocity/cmd_vel
/uav1/mavros/setpoint_velocity/cmd_vel_unstamped
/uav1/mavros/state
/uav1/mavros/statustext/recv
/uav1/mavros/statustext/send
/uav1/mavros/target_actuator_control
/uav1/mavros/time_reference
/uav1/mavros/timesync_status
/uav1/mavros/trajectory/desired
/uav1/mavros/trajectory/generated
/uav1/mavros/trajectory/path
/uav1/mavros/vfr_hud
/uav1/mavros/vision_pose/pose
/uav1/mavros/vision_pose/pose_cov
/uav1/mavros/vision_speed/speed_twist_cov
/uav1/mavros/wind_estimation
```

## Extra
---
**1. Terminator**

To make life easier, you can install terminator, an alternative terminal with additional features and functionality

To install, enter these commands:
```
sudo add-apt-repository ppa:gnome-terminator
sudo apt-get update
sudo apt-get install terminator
```
You can open terminator up with `ctrl + alt + t`

To split into multiple terminals, use `ctrl + shift + o` or `ctrl + shift + o`

When you have your desired layout, right click on any terminal, click on **Preferences**, Go to the Layouts tab, add new layout, rename as **default**, click save, then close.

Restart terminator and you should have your desired layout upon startup

**2. Aliases**

With aliases, you can run the programs quicker without having to type out the whole command. Here is how to add aliases:

1. Go to the home directory and open up the .bashrc file
```
cd ~
gedit .bashrc
```
2. Add the following aliases at the bottom of the file
```
# Easier navigation
alias ..="cd .."
alias ...="cd ../.."
alias ....="cd ../../.."
alias ~="cd ~"
alias src="source ~/.bashrc"
alias c="cd ~/catkin_ws && catkin build"
alias cs="cd ~/catkin_ws/src && source ~/catkin_ws/devel/setup.bash"
alias bs="gedit ~/.bashrc"

# PX4
alias m="cd ~/PX4/PX4-Autopilot && make"
alias p="cd ~/PX4 && ./px4.sh"
alias qgc="cd ~/PX4 && ./QGroundControl.AppImage"

# ROS
alias rr="rosrun continuous_yaw continuous_yaw"
alias mrl="roslaunch mask_rcnn_ros rcnn_track.launch"
alias yl="roslaunch yolo_ros yolo_track.launch"

# git commands
alias st="git status"
alias add="git add ."
alias push="git push"

# virtual env
alias cnn="conda activate my_env"
alias yolo="conda activate yolo"
alias deact="conda deactivate"
```

3. Restart terminator


Now that you have added these aliases, if you want to run the `px4.sh` shell script, instead of going to the terminal to type `cd ~/PX4 && ./px4.sh`, you can now just type `p` and enter

**3. Visual Code Studio**

You can download VCS so that you can edit/read/write codes more easily. To do so:
1. Go to https://code.visualstudio.com/ and download vcs

2. Open the PX4/PX4-Autopilot folder

3. Install all workspace extension recommendations

**Note: Do not install a newer version of Cmake as the correct version has already been installed**
## References
---
PX4 setup: http://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html

QGroundControl: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

Drone tutorials (Ardupilot): https://github.com/Intelligent-Quads/iq_tutorials
