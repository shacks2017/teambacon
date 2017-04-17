#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
int i=0;
int a=0;
int b=0;
int c=0;
float dis=-1;
float dis2=-1;
bool g_lidar_alarm=false;
void alarmCallback(const std_msgs::Bool& alarm_msg) 
{
	g_lidar_alarm = alarm_msg.data; //Sets g_lidar_alarm to the data from the subscriber
} 

void disback(const std_msgs::Float32& dis_msg)
{
  dis=dis_msg.data;
 // if(c==0){
 // dis2=dis;
 // c=1; 
 // }
  ROS_INFO("the minimum distance is %f",dis);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_detecting");
    ros::NodeHandle n;
    ros::Rate looprate(5.0);
        ros::Subscriber degree_subscriber = n.subscribe("lidar_dis",1,disback);
	ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback); 
	ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("estop_service");
	ros::ServiceClient client2 = n.serviceClient<std_srvs::Trigger>("clear_estop_service");
	std_srvs::Trigger srv; //Sets up estop.

	while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");

    while (!client2.exists()) {
      ROS_INFO("waiting for service2...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client2 to service");
   while(ros::ok())
    {
          if(g_lidar_alarm==true&&b==0){
                client.call(srv);
                ROS_WARN("obstacles ahead");
                a=1;b=1;
                
           }
          
     
          if(g_lidar_alarm==false&&a==1){
                client2.call(srv);
                a=0;b=0;
                ros::Duration(3.0).sleep();
           }

   //       dis2=dis;
          ros::spinOnce();
    }

	return 0;//Should not get here  
}
