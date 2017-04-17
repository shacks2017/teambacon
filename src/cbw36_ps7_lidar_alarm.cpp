// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE = 2.0; // set alarm if anything is within 0.5m of the front of robot
const double R=0.25;  //radius of robot plus cushion

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int min_ping_index_= -1; // NOT real; callback will have to find this
int max_ping_index_= -1;
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;


void laserCallback(const sensor_msgs::LaserScan& laser_scan) 
{
  if (min_ping_index_<0)  
  {
    angle_min_ = laser_scan.angle_min;
    angle_max_ = laser_scan.angle_max;
    angle_increment_ = laser_scan.angle_increment;
    range_min_ = laser_scan.range_min;
    range_max_ = laser_scan.range_max;

    double theta_=atan(MIN_SAFE_DISTANCE/R);  //calculate angle at which point if a laser_scan were to drop a perpendicular of length min_safe_dist it would be tangent to R
    double phi_=atan(1)*4-theta_;  //Calculate how far off it is from the angle=0=direction forward by subtacting pi/2
      


    min_ping_index_=0;
    max_ping_index_=10;
    // min_ping_index_ = (int) ((-phi_ -angle_min_)/angle_increment_);  //calculate phi degrees left of straight forwards motion vector
    // max_ping_index_ = (int) ((phi_ - angle_min_)/angle_increment_);  //calculate phi degrees right of straight forwards motion vector
    ROS_INFO("LIDAR setup: ping_index_range = %d, %d",min_ping_index_, max_ping_index_);
  }
    
   
  for( int index=min_ping_index_; index<max_ping_index_; index=index+1)  //iterate from min ping to max ping
  {
    ping_dist_in_front_ = laser_scan.ranges[index];  //if any have range values less than min_safe_dist return DANGER
   	ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   	if (ping_dist_in_front_<MIN_SAFE_DISTANCE) 
    {
   	  ROS_WARN("DANGER, WILL ROBINSON!!");
   	  laser_alarm_=true;
  	  index=max_ping_index_;
	  }
	  else 
    {
      laser_alarm_=false;
		}
  } 


   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);  
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    lidar_alarm_publisher_ = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);

    lidar_dist_publisher_ = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}