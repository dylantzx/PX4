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
## To Run Gazebo Simulation with MavROS
---
1. Change into PX4 directory and run script
```
cd PX4
./px4.sh 
```
The command runs the script in a child terminal. If you want to run the script in your current terminal, you can run
```
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
/mavlink/from
/mavlink/gcs_ip
/mavlink/to
/mavros/actuator_control
/mavros/adsb/send
/mavros/adsb/vehicle
/mavros/altitude
/mavros/battery
/mavros/cam_imu_sync/cam_imu_stamp
/mavros/companion_process/status
/mavros/debug_value/debug
/mavros/debug_value/debug_vector
/mavros/debug_value/named_value_float
/mavros/debug_value/named_value_int
/mavros/debug_value/send
/mavros/esc_info
/mavros/esc_status
/mavros/estimator_status
/mavros/extended_state
/mavros/fake_gps/mocap/tf
/mavros/geofence/waypoints
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/gp_origin
/mavros/global_position/home
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/rel_alt
/mavros/global_position/set_gp_origin
/mavros/gps_rtk/rtk_baseline
/mavros/gps_rtk/send_rtcm
/mavros/gpsstatus/gps1/raw
/mavros/gpsstatus/gps1/rtk
/mavros/gpsstatus/gps2/raw
/mavros/gpsstatus/gps2/rtk
/mavros/hil/actuator_controls
/mavros/hil/controls
/mavros/hil/gps
/mavros/hil/imu_ned
/mavros/hil/optical_flow
/mavros/hil/rc_inputs
/mavros/hil/state
/mavros/home_position/home
/mavros/home_position/set
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/diff_pressure
/mavros/imu/mag
/mavros/imu/static_pressure
/mavros/imu/temperature_baro
/mavros/imu/temperature_imu
/mavros/landing_target/lt_marker
/mavros/landing_target/pose
/mavros/landing_target/pose_in
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/log_transfer/raw/log_data
/mavros/log_transfer/raw/log_entry
/mavros/manual_control/control
/mavros/manual_control/send
/mavros/mission/reached
/mavros/mission/waypoints
/mavros/mocap/pose
/mavros/mount_control/command
/mavros/mount_control/orientation
/mavros/obstacle/send
/mavros/odometry/in
/mavros/odometry/out
/mavros/onboard_computer/status
/mavros/param/param_value
/mavros/play_tune
/mavros/px4flow/ground_distance
/mavros/px4flow/raw/optical_flow_rad
/mavros/px4flow/raw/send
/mavros/px4flow/temperature
/mavros/radio_status
/mavros/rallypoint/waypoints
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/setpoint_accel/accel
/mavros/setpoint_attitude/cmd_vel
/mavros/setpoint_attitude/thrust
/mavros/setpoint_position/global
/mavros/setpoint_position/global_to_local
/mavros/setpoint_position/local
/mavros/setpoint_raw/attitude
/mavros/setpoint_raw/global
/mavros/setpoint_raw/local
/mavros/setpoint_raw/target_attitude
/mavros/setpoint_raw/target_global
/mavros/setpoint_raw/target_local
/mavros/setpoint_trajectory/desired
/mavros/setpoint_trajectory/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/mavros/statustext/recv
/mavros/statustext/send
/mavros/target_actuator_control
/mavros/time_reference
/mavros/timesync_status
/mavros/trajectory/desired
/mavros/trajectory/generated
/mavros/trajectory/path
/mavros/vfr_hud
/mavros/vision_pose/pose
/mavros/vision_pose/pose_cov
/mavros/vision_speed/speed_twist_cov
/mavros/wind_estimation
/rosout
/rosout_agg
/tf
/tf_static
```

## Extra
---
**1. Terminator**

To make live easier, you can install terminator, an alternative terminal with additional features and functionality

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


## References
---
PX4 setup: http://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html

QGroundControl: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

Drone tutorials (Ardupilot): https://github.com/Intelligent-Quads/iq_tutorials