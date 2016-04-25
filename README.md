## KUKA LBR iiwa Camera Calibration Tool

This package is a ROS extension for the [Camera-to-Arm tool](https://github.com/ZacharyTaylor/Camera-to-Arm-Calibration) developed by Zachary Taylor. It tries to reduce the headache of manually recording the end-effector to robot transformation at each calibration pose and command the ROS node to capture the current image and save the current end-effector pose.
<!--more-->
This package is mainly implemented for KUKA LBR iiwa R820. But theoretically it can be modified with ease to fit any manipulator. Just change the image topic name and find a way to transmit a pose at each calibration view.


### *To run on KUKA LBR R820:*

#### Sunrise side:

Add the [CameraCalib.jave](https://github.com/elhussieny/kuka_camera_calib/tree/master/javaNode) file to the robot applications in Sunrise OS. Complie and synchronize with the KUKA Cabinet. This file is in charge of sending the calibration poses of the end-effector to the ROS package. Follow the instructions on the SamrtPAD, enable the CCP user defined key and move the robot to a suitable place in front of the mounted camera. To save a configuration just press the user defined CCP. Don't press CCP again untill runing the ROS package. 

#### roscore side:
1. Download and install ROS Disrtibution.

2. Download the kuka_camera_calib package in the ros workspace **~/ROS_ws/src/**
    ```
    git clone https://github.com/elhussieny/    kuka_camera_calib
    ```

3. catkin_make the workspace:
	```
	catkin_make
	```

4. Creat an empty directory *Calibration/Images* inside the kuka_camera_calib directory:
	```
	mkdir -p Calibration/Images
	```

5. Run the image streaming node; for example kinect2 bridge that will publish image topics:
	```
	roslaunch kinect2_bridge kinect2_bridge
	```

6. Run the kuka_camera_calib node:
	```
	rosrun kuka_camera_calib kuka_calib
	```
7. The image should be viewed in a window. Pressing the CCP on the KUKA SmartPAD will save the current pose of the end-effector and also save the current displayed image. The final file that contains all the poses will be saved in **calib_xxx.csv** file in the kuka_camera_calib/Calibration file. 

#### MATLAB side:

1. Open MATLAB 8.0+ and jump to the **kuka_camera_calib/matlabCode/** and run the following command in the matlab command window:

    ```
    calibrateKUKA  ('filename.csv',chess_square_ width_in_mm)
    ``` 

    for instance, invoke it in this way:
    ```
    calibrateKUKA   ('calib_1461440868.500580.csv',27.5)
    ``` 

2. Wait till the code finished and check these outputs:

**TBase**- The camera to arm base transformation.
**TEnd**- The end effector to checkerboard transformation.
**cameraParams**- The intrinsic parameters of the camera used.
**TBaseStd**- An estimation of the standard deviation of the error in the values in TBase.
**TBaseStd**- An estimation of the standard. deviation of the error in the values in TEnd.
**pixelErr**- Mean error of projected inliers in pixels.

