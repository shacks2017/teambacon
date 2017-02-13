// PS2, based on wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h>       /* acos */
#include <cmath>
#include <unistd.h>
#define PI 3.14159265


const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 1m of the front o

// these values to be set within the laser callback
int zero_index = -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

double robot_radius=0.1; //.2m radius for robot0; should probably be picking this up from a subscriber...

//New parameters based on robot width:
int ping_min=0;
int ping_max=0;

// angle and ray length parameters
double theta, r;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

float index_to_angle(float i)
{
  return (i - zero_index) * angle_increment_; //helper function to calculate angle of ray
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {

    if(zero_index < 0) {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        zero_index = (int)(-angle_min_ / angle_increment_);

        //theta = PI/2 - acos((robot_radius+.2)/MIN_SAFE_DISTANCE); \\used for "pizza slice" method...old code
        //hypotenuse=sqrt(2*pow(robot_radius,2));
        ping_min = zero_index - (int)((PI/16)/angle_increment_);
        ping_max = zero_index + (int)((PI/16)/angle_increment_);

        ROS_INFO("LIDAR setup: zero_index = %d", zero_index);
    }

    laser_alarm_=false; //reset laser alarm; keep out of for loop
    for(int ping_index = ping_min; !laser_alarm_ && ping_index <= ping_max; ++ping_index){ //break out of for loop if alarm goes off
      theta = index_to_angle(ping_index); //angle of ray
      r     = laser_scan.ranges[ping_index]; //distance of ray
      if(abs(r * sin(theta)) > robot_radius) //left/right bracketing; r*sin(theta)=x
        continue;
      if(r < MIN_SAFE_DISTANCE) 
        laser_alarm_ = true; //triggers alarm if any ray is too close
    }
    if(laser_alarm_)
      ROS_INFO("Collision detected in %f meters at angle %f", r, theta);
    else
      ROS_INFO("No collision.");

    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    /*std_msgs::Float32 lidar_dist_msg;
    lidar_dist_msg.data = clearance;
    lidar_dist_publisher_.publish(lidar_dist_msg); */
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kvc2_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

