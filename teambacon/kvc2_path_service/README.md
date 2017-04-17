# kvc2_path_service

Mobile Robotics, Spring 2017, PS3. Spawn a robot in 2D simulator with predefined map, create an open loop instruction set to move it to the top left corner, executed through a ROS service. 

## Example usage
In a terminal window, type:
	roscd
	roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch &
	rosrun kvc2_path_service kvc2_path_service &
	rosrun kvc2_path_service kvc2_path_client

    
