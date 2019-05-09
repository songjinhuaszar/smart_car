//
// Created by jason on 4/11/18.
//
#include "SerialClass.hpp"
SerialClass *serClass;

int main(int argc,char *argv[])
{
   	 ros::init(argc,argv,"smart_car");
     ros::NodeHandle nh;
   	 ros::Subscriber cmd = nh.subscribe<smart_car::smart_car_cmd>("smart_car_cmd",1000,SerialClass::callback);//订阅smart_car_cmd主题

	ros::Subscriber sub = nh.subscribe("cmd_vel", 20, SerialClass::callback_geometry); //订阅cmd_vel主题

	nav_msgs::Odometry odom;//定义里程计对象
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布odom主题

	static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息

	//-------超声传感器信息
	sensor_msgs::Range range_msg;
	ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>("sonar",20);

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND; 
	range_msg.header.frame_id ="sonar"; 
	range_msg.field_of_view = 0.5; 
	range_msg.min_range = 0.05; 
	range_msg.max_range = 2;

	//-----serialPort
   	 string port = "/dev/ttyUSB1";
    	int baudrate = 115200;
   	 if(ros::param::has("~port_name")){
       		 ros::param::get("~port_name",port);
   	 }
    	if(ros::param::has("~baudrate")){
       		 ros::param::get("~baudrate",baudrate);
   	 }
    	ROS_INFO_STREAM(port);

    serClass = new SerialClass(port,baudrate);

	int ncounter = 0;
    ros::Rate loop_rate(50); 
	while(ros::ok())
	{
		//---serialwork
		ros::spinOnce();

		//---
		// ncounter++;
		// if(ncounter >= 500)
		// {
		// 	serClass->sendCmd();
		// 	ncounter = 0;
		// }

		size_t len = serClass->ser.available();
		if(len){
		    serClass->recvData(len);
		}
		
		 //---pub tf(odom->base_footprint)
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";
            odom_trans.transform.translation.x = serClass->myOdomData.position_x;
            odom_trans.transform.translation.y = serClass->myOdomData.position_y;
            odom_trans.transform.translation.z = serClass->myOdomData.position_z;
            odom_trans.transform.rotation.x = serClass->myOdomData.orientation_x;
            odom_trans.transform.rotation.y = serClass->myOdomData.orientation_y;
            odom_trans.transform.rotation.z = serClass->myOdomData.orientation_z;
            odom_trans.transform.rotation.w = serClass->myOdomData.orientation_w;

		//---pub odom
		
           	odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";
			odom.pose.pose.position.x = serClass->myOdomData.position_x;
			odom.pose.pose.position.y = serClass->myOdomData.position_y;
			odom.pose.pose.position.z = serClass->myOdomData.position_z;
			odom.pose.pose.orientation.x = serClass->myOdomData.orientation_x;
			odom.pose.pose.orientation.y = serClass->myOdomData.orientation_y;
			odom.pose.pose.orientation.z = serClass->myOdomData.orientation_z;
			odom.pose.pose.orientation.w = serClass->myOdomData.orientation_w;
			odom.twist.twist.linear.x = serClass->myOdomData.linear_x;
			odom.twist.twist.linear.y = serClass->myOdomData.linear_y;
			odom.twist.twist.linear.z = serClass->myOdomData.linear_z;
			odom.twist.twist.angular.x = serClass->myOdomData.angular_x;
			odom.twist.twist.angular.y = serClass->myOdomData.angular_y;
			odom.twist.twist.angular.z = serClass->myOdomData.angular_z;
           
			
			odom_broadcaster.sendTransform(odom_trans);
			odom_pub.publish(odom);
		
		//---

			range_msg.range = serClass->myRange;
			pub_range.publish(range_msg);
			
		//-----
		loop_rate.sleep();
		
	}
    return 0;
}
