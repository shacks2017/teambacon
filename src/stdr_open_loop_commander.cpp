#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds
    
      
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;   

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }
    twist_cmd.linear.x=speed; //command to move forward
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

    twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again
    timer=0.0; //reset the timer
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    //halt the motion
    twist_cmd.angular.z=0.0; 
    twist_cmd.linear.x=0.0; 
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }               
    // Above is sample code; the new code begins here.  .................................................
    

    twist_cmd.linear.x=speed; //command to move forward
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }



    twist_cmd.linear.x=speed; //command to move forward
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<9.5){ //Turn 270 degrees instead of 90...
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again - to the right. Eastward. 
    timer=0.0; //reset the timer
    while(timer<3.4) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    //halt the motion

    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<3) { //90 deg turn to go north, above the thick black bar
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

    twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again, northward
    timer=0.0; //reset the timer
    while(timer<2.1) { //move forward
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }      
    //halt the motion

    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<2.6) { //Turn to go west, above the thick black bar - manually tuned
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

    twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again, westward
    timer=0.0; //reset the timer
    while(timer<1) { //move forward
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }      
    //halt the motion


   twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<.75) { //slight course correction...
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

    twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again
    timer=0.0; //reset the timer
    while(timer<3.1) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }      
    //halt the motion

   twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<11.5) { //Turning back north by northwest...
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

    twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again
    timer=0.0; //reset the timer
    while(timer<2.1) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }      
    //halt the motion
   
        twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<11){ //Another ~360 degree turn...
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and straight on 'til morning...!
    timer=0.0; //reset the timer
    while(timer<3.2) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }

// By this point, the 'bot may already be at the top corner; but just in case, we'll have it turn a bit CW and go forward. 
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<11.85){ //Another ~360 degree turn...
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and straight on 'til morning...!
    timer=0.0; //reset the timer
    while(timer<3) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    //halt the motion
}

