# Visual Odometry Based on RGB images

This packages listens to an RGB image topic and calculates the camera motion on a 2D plane based on feature tracking 
(Lukas-Kanade method) and optical flow. 

It requires a calibrated RGB camera with its parameters published on the ROS parameter server. An example file can be 
found in the [config](./config) folder.

By default the node listens for images in the ```/usb_cam/image_raw``` topic and publishes the estimated motion as a 
```geometry_msgs/Twist``` message on the ```/twist/visual```
#### Example Run
```bash
user@host:~$ roslaunch visual_odometry usb_cam.launch 
```

#### Depends
- ROS Kinetic
- OpenCV 2 and greater
- ROS Image Transport

#### Generate Documentation
```bash
user@host:~$ doxygen Doxyfile
```

#### TODO
- Implement some of the camera attribute getters/setters 
- Try different motion tracking methods
