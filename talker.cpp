#include "ros/ros.h"
#include "smart_car/smart_car_cmd.h"
#include "SerialClass.hpp"
#include <cstdlib>
#include <std_msgs/String.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

 	ros::Publisher command_pub = n.advertise<smart_car::smart_car_cmd>("smart_car_cmd",1000);
	ros::Rate loop_rate(50);
	int count = 1;
//---
	 string str_cmd = "Read";
     int nvalue = 0;
   	 if(ros::param::has("~str_cmd")){
       		 ros::param::get("~str_cmd",str_cmd);
   	 }
     if(ros::param::has("~nvalue")){
       		 ros::param::get("~nvalue",nvalue);
   	 }
		smart_car::smart_car_cmd msg;
		msg.msgtype = str_cmd.c_str();
		msg.value=nvalue;
//---
	while(ros::ok())
	{

		// /
		
		// std::string str;
		// str = str_cmd;
		
		// msg.msgtype = str.c_str();
		// msg.value = nvalue;
		// std_msgs::String msg;
		// msg.data = str_cmd.c_str();

		ROS_INFO("Talker: msgtype = %s,value = %d",msg.msgtype.c_str(),msg.value);
		
		command_pub.publish(msg);
	
		loop_rate.sleep();

	}

  	return 0;
}
