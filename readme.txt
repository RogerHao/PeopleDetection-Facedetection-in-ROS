this is a project to detect pedastrain and face on ROS

image_node_a : read a movie clip and publish the frame
image_node_b : subcribe the frame of node_a and detect the people, then publish the ROI(people) Mat
fdetection : subscribe the ROI Mat from node_b and detect the face, then show the result

ros_version : kinetic
opencv_version : opencv-3.2.0.dev
