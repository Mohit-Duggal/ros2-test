# ros2-test

ubuntu 20.4 X64
Make sure you install  ros2 usb_cam package with ROS 2 distro installed 
sudo apt-get install ros-humble-usb-cam
if you install with Building from Source 
    there should be some error
    
In my system usb_cam package dont work with laptop cam 
I use external camera
##run the node

##Publish an image
 ros2 run usb_cam usb_cam_node_exe

##change the mode
  ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"  # Grayscale mode
  ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}" # Color mode

ros2 run rqt_image_view rqt_image_view
