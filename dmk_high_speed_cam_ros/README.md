# dmk_high_speed_cam_ros
This is a package to run the Imaging source dmk series under ROS.

# Building tiscamera

The following commands will build and install our software with default settings.

```
git clone https://github.com/TheImagingSource/tiscamera.git
cd tiscamera
git checkout v-tiscamera-1.0.0
# only works on Debian based systems like Ubuntu
sudo ./scripts/dependency-manager install
mkdir build
cd build

# With ARAVIS:
cmake -DBUILD_ARAVIS=ON ..

make
sudo make install
```

# Usage
```bash
roslaunch dmk_high_speed_cam_ros high_speed_cam.launch cam_id:=SN
```
- Please check `SN` by using tcam-capture or the tape on the camera, this should be a digital number (e.g., 4220614).

# Reference
You can find an online version of the included user documentation here:

https://www.theimagingsource.com/documentation/tiscamera/

https://www.argocorp.com/cam/usb3/tis/DFK_DMK.html
