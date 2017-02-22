# bacon1
Package containing lab exercises for EECS 476. 

# Example usage
First, connect to Turtlebot using the instructions in the lab assignment. Recommended to use Google's DNS for stability. Also, note that connecting to the Turtlebot in one terminal does NOT connect you to it in other terminals. To wit, for deeplearning02:

Find IP address using Google DNS and write it down...
host deeplearning02w 8.8.8.8 

Find IP address for desktop and write it down
ifconfig 

Ping and ensure data return. Can replace hostname with IP address - recommended, even...
ping deeplearning02w 8.8.8.8 

Export ROS_MASTER_URI... can replace hostname with IP address
export ROS_MASTER_URI=http://deeplearning02w.eecs.cwru.edu:11311

Identify your local ROS IP:
export ROS_IP=XXX.XX.XXX.XX
Verify connection by checking rostopics:
rostopic list

Next, return the source to the .src file:
source /opt/ros/indigo/setup.bash
...and run the program:
rosrun bacon1 bacon_commander (e.g.)

Keep an eye on the turtlebot and make sure it doesn't bump into anything!

# Running tests/demos

## PS1: Open Loop Control
rosrun bacon1 bacon_commander
Can change standard velocities &c in .cpp file. Be sure to keep velocity relatively low. Note that, because the controller is open loop, the robot may not circumscribe a very precise square. 

## PS2: Reactive control with LIDAR
rosrun bacon1 bacon_lidar
rosrun bacon1 bacon_reactive

bacon_reactive must be used because it publishes to the topic /cmd_vel, rather than the topic /robot0/cmd_vel (as would be required for STDR).

## PS3: Path Server
rosrun bacon1 bacon_path_service &
rosrun bacon1 bacon_path client
