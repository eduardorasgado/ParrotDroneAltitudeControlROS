# ParrotDroneAltitudeControlROS
A ROS and Gazebo Parrot Drone altitude control simulation based on Action client-server communication.

For simulation:\
	roslaunch ardrone_altitude ardrone_altitude_server.launch\
	roslaunch ardrone_altitude ardrone_altitude_client.launch

After doing this just write in "" your UP or down in client.\
I meant this: "UP", "DOWN"\
For finishing the fight:\
	rostopic pub /drone/land std_msgs/Empty "{}"
