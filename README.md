# ros2-test

Installing the usb_cam Package:
Ensure that you install the usb_cam package for your ROS 2 distribution using the following command:
sudo apt-get install ros-humble-usb-cam
If you try to install it by building from source, you might encounter some errors.

Camera Compatibility:
On my system, the usb_cam package doesn't work with the built-in laptop camera, so I use an external camera instead.

Running the Node:
To publish an image from the external camera, use this command:
ros2 run usb_cam usb_cam_node_exe
Changing Camera Mode:
You can toggle between grayscale and color modes using the following commands:

Grayscale mode:
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
Color mode:
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
Viewing the Image:
To view the published images, use the following command:
ros2 run rqt_image_view rqt_image_view
