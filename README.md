# ros2-test

ubantu 24.4
ros-jazzy

## Running the Node:

ros2 launch image_conversion_cpp image_conversion

## To publish an image from the camera, use this command:
ros2 run usb_cam usb_cam_node_exe  .
## Changing Camera Mode:
You can toggle between grayscale and color modes using the following commands:

# Grayscale mode:
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
# Color mode:
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
Viewing the Image:
# To view the published images, use the following command:
ros2 run rqt_image_view rqt_image_view
