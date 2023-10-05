# Unitree Camera Streaming
Author: Katie Hughes

This is a ROS 2 package using the standard image pipeline to stream camera info from a Unitree Go1 model. 

Before you are able to stream, the cameras must first be unlocked.

To stream from the onboard cameras run 
`ros2 launch stream_camera start_camera.launch.xml`. This will instantiate `usb_cam` nodes with the appropriate video parameters.

The node `split_image` will divide the published usb image in half so that stereo video processing can continue.