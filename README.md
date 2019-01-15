This is the catkin workspace of TX1


*********************************** IMPORTANT **************************************************************

THIS WILL REQUIRE THE FOLLWING ROS PACKAGES TO RUN

ZED wrapper:-

http://wiki.ros.org/zed-ros-wrapper

Vectornav wrapper:-

https://github.com/dawonn/vectornav



**************************************************************************************************************


To import the code into required ROS system do the following:-

1. INSTALL ABOVE MENTIONED PACKAGES

2. CLONE THE REQUIRED PACKAGE INTO catkin/src

3. OPEN A TERMINAL IN ~/catkin

4. RUN THE FOLLOWING COMMAND:-
   
   catkin_make
   
5. RUN THE REQUIRED PAKAGE WITH THE rosrun COMMAND. Example:-
 
   rosrun ip_zed zed_depth.py
   
   
   
   