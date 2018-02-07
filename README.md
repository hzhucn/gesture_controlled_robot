# Gesture Robot Control

Final year project part of the image and machine learning courses. The aim was to design classifiers and a ROS architecture to command khepera / p3dx robots

### Description of the prototype
After preprocessing the image, we extracted fourier descriptors from the contour of the hand to feed a classifier. We tried several types of supervised vs unsupervised classifiers (take a look at our benchmark by running the corresponding notebook) and eventually 
we chose 2 of them : a 4-clusters KMeans to reject non-hand images and a MLP classifier to predict the class corresponding to the image.

### Robot Command
Thanks to the chosen classifiers, we created 3 types of commands : 
- a Telnet command to guide the robot directly thanks to TCP requests (file robot_command/Telnet)
- a ROS simulation using VREP and a scene given on during the laboratory work on autonomous robotics
- a ROS command to guide the KheperaIV robot
