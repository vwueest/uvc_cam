# UVC USB camera driver for ROS
Simple UVC USB camera driver for ROS, based on Fabian Schilling's [OpenMV Cam ROS package](https://github.com/fabianschilling/openmv_cam). The standard libuvc did not allow me to set all settings as I wished, with this package this problem was solved.

First, install 
```
sudo apt-get install autoconf libtool nasm
```
Then, install the UVC library for python, as described in their README with all dependencies, clone it and instal it,
```
https://github.com/pupil-labs/pyuvc
cd ~/catkin_ws/src/
git clone git@github.com:pupil-labs/pyuvc.git
cd pyuvc
pip install -e .
```
Then, clone the repo,
```
cd ~/catkin_ws/src
git clone https://github.com/vwueest/uvc_cam.git
```
Finally, run the launch file,
```
roslaunch uvc_cam camera.launch 
```
