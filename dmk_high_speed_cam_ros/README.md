# dmk_high_speed_cam_ros
This is a package to run the Imaging source dmk series under ROS.

# Requirement
* Ubuntu 18.04
* ROS melodic

# Building tiscamera

The following commands will build and install our software with default settings.

```
git clone https://github.com/TheImagingSource/tiscamera.git
cd tiscamera
# only works on Debian based systems like Ubuntu
sudo ./scripts/install-dependencies.sh --compilation --runtime
mkdir build
cd build

# With ARAVIS:
cmake -DBUILD_ARAVIS=ON ..
# Without ARAVIS
cmake -DBUILD_ARAVIS=OFF ..

make
sudo make install
```

# Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/shuto1441/dmk_high_speed_cam_ros.git
cd ../
catkin build
```

# Usage
```bash
roslaunch dmk_high_speed_cam_ros high_speed_cam.launch
```

# Reference
You can find an online version of the included user documentation here:

https://www.theimagingsource.com/documentation/tiscamera/

https://www.argocorp.com/cam/usb3/tis/DFK_DMK.html
