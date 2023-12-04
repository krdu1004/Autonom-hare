# Autonom-hare
Autonom hare


- Change the variable videofolder on line 59 in main.py to the folder containing the images you want to detect lines on. 
- The ouput is an in image called image that contains left and right lane lane, as well as a center line for the current lane
of the camera. Steering angle is the controll output signal based on the angle to the vanishing point found from the
intersection of left and right lane and the distance between center of input image and center of lane. 
Both of these references can be set by changing *angle_setpoint* and *position_endpoint*
- There are currently 6 different techniques for line fitting implemented. Currently all of them are plotted, remove unwanted methods.
- *set_width* reshapes the image to this width.
- *cut_top* removes this percentage from the top of the image. Used to remove everything above the running track.


main_with_prints_and_plot.py contains lots of other plots, print statements for making results and debugging. 

