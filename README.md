## KUKA LBR iiwa Camera Calibration Tool

This package is a ROS extension for the [Camera-to-Arm tool](https://github.com/ZacharyTaylor/Camera-to-Arm-Calibration) developed by Zachary Taylor. It tries to reduce the headache of manually recording the end-effector to robot transformation at each calibration pose and command the ROS node to capture the current image and save the current end-effector pose.
<!--more-->
This package is mainly implemented for KUKA LBR iiwa R820. But theoretically it can be modified with ease to fit any manipulator. Just change the image topic name and find a way to transmit a pose at each calibration view.


## *To run on KUKA LBR R820:*




 **The pose should be capture in this format:**

[pose_no, Xe, Ye, Ze, Alpha, Beta, Gamma];

