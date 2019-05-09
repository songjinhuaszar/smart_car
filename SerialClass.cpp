
#include "ros/ros.h"
#include "SerialClass.hpp"
extern SerialClass * serClass;

SerialClass::SerialClass(string ttyName,int Baudrate,int timeout)
{
    this->ttyName = ttyName;
    this->Baudrate = Baudrate;
    this->timeout = timeout;
    this->data = new unsigned char[20];
   
    Address =-1;
    try{
        ser.setPort(ttyName);
        ser.setBaudrate(Baudrate);
        ser.setParity(parity_none);
        ser.setBytesize(eightbits);
        ser.setStopbits(stopbits_one);
        ser.setFlowcontrol(flowcontrol_none);
        Timeout o = Timeout::simpleTimeout(timeout);
        ser.setTimeout(o);
        ser.open();
    }catch (IOException &e){
        ROS_ERROR_STREAM("unable to open port");
        return ;
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port was opened");
    }

}

void SerialClass::callback(const smart_car::smart_car_cmd::ConstPtr msg)
{
    ROS_INFO_STREAM("receive topic");
    serClass->sendCmd(msg);
}

void SerialClass::run()
{
  /*  ros::Rate rate(200);
    rate.sleep();
   
    ros::Rate loop_rate(1);
   
    while(ros::ok()){

        ros::spinOnce();
        size_t len = ser.available();
        if(len){
            recvData(len);
        }
        loop_rate.sleep();

	

    }*/
}

void SerialClass::callback_geometry(const geometry_msgs::Twist & cmd_input)
{
	
	ROS_INFO_STREAM("receive topic___cmd_vel !");
    
	int Value[6] = {0};
		

	Value[0] = cmd_input.linear.x * 1000;
	Value[1] = cmd_input.linear.y * 1000;
	Value[2] = cmd_input.linear.z * 1000;
	Value[3] = cmd_input.angular.x * 1000;
	Value[4] = cmd_input.angular.y * 1000;
	Value[5] = cmd_input.angular.z * 1000;

//	printf("linear.x:%f \n",cmd_input.linear.x);

	for (size_t i = 0; i < 6; i++)
	{
		if(Value[i]<0)
		{
			Value[i] = Value[i] + 65536;
		}
	}
	
	serClass->SendSpeed((unsigned int*)Value);
	
}
unsigned char SerialClass::SendSpeed(unsigned int *Value)
{
	unsigned char Data_Length = 0;
	unsigned char Data[100] = { 0 };

	
	Data_Length = protocol.Dynamixel_Send(0x03, 0x09, Value, 12, Data);
	size_t  sendlen = ser.write(Data, Data_Length);
	ROS_INFO("send datalen is %d",sendlen);

	ROS_INFO_STREAM("\n");

	return Dynamixel_State_Success;

}

unsigned char SerialClass::sendCmd(const smart_car::smart_car_cmd::ConstPtr msg)
{
	
	int number = -1;

	unsigned char Data_Length = 0;
	unsigned char Data[100] = { 0 };
	unsigned int Value[6];
	Value[0] = 10;
	Value[1] = 0;
	Value[2] = 0;
	Value[3] = 0;
	Value[4] = 0;
	Value[5] = 0;

    if(msg!=NULL){
		
		ROS_INFO("the cmd is %s,the value is %d",msg->msgtype.c_str(),msg->value);
		number = msg->value;

		if(msg->msgtype=="Read")
		{
			Data_Length = protocol.Dynamixel_Send(0x02, 1, Value, 70, Data);
		
			 size_t  sendlen = ser.write(Data, Data_Length);
	   		 ROS_INFO("send datalen is %d",sendlen);
			
		}
		if (msg->msgtype=="WriteR")
		{
			Value[0] = 150;
			Data_Length = protocol.Dynamixel_Send(0x03, 0x02, Value, 2, Data);
			 size_t  sendlen = ser.write(Data, Data_Length);
	   		 ROS_INFO("send datalen is %d",sendlen);

		}

		if (msg->msgtype=="WriteSpeedL")
		{
			Value[0] = 10;
			Data_Length = protocol.Dynamixel_Send(0x03, 0x09, Value, 2, Data);
			 size_t  sendlen = ser.write(Data, Data_Length);
	   		 ROS_INFO("send datalen is %d",sendlen);

		}
		if (msg->msgtype=="WriteSpeedM")
		{
			Value[0] = 0;
			Data_Length = protocol.Dynamixel_Send(0x03, 0x09, Value, 12, Data);
			size_t  sendlen = ser.write(Data, Data_Length);
	   		ROS_INFO("send datalen is %d",sendlen);

		}
		if (msg->msgtype=="WriteSpeedH")
		{
			Value[0] = 100;
			Data_Length = protocol.Dynamixel_Send(0x03, 0x09, Value, 2, Data);
			size_t  sendlen = ser.write(Data, Data_Length);
	   		ROS_INFO("send datalen is %d",sendlen);

		}
		if (msg->msgtype=="WriteStop")
		{
			Value[0] = 0;
			Data_Length = protocol.Dynamixel_Send(0x03, 0x09, Value, 2, Data);
			size_t  sendlen = ser.write(Data, Data_Length);
	   		ROS_INFO("send datalen is %d",sendlen);

		}
		if (msg->msgtype=="WriteV")
		{
			Value[0] = 1;
			Data_Length = protocol.Dynamixel_Send(0x03, 0x01, Value, 1, Data);
			size_t  sendlen = ser.write(Data, Data_Length);
	   		ROS_INFO("send datalen is %d",sendlen);
		}
       
    }else
	{
		Data_Length = protocol.Dynamixel_Send(0x02, 1, Value, 70, Data);
		
		size_t  sendlen = ser.write(Data, Data_Length);
		ROS_INFO("sending default data!");
	}

	ROS_INFO_STREAM("\n");
	return Dynamixel_State_Success;
}

unsigned char SerialClass::recvData(size_t len)
{
    unsigned char * data = new unsigned char[len]{0};
    size_t recvlen = ser.read(data,len);
   
    ROS_INFO_STREAM("Ros Receive!");
    ROS_INFO("rece data,the data len is %d;recvlen is %d",len,recvlen);
 //   for(int i=0;i<len;i++){
   //     ROS_INFO("OX%02x",data[i]);
 //   }
	
    if(protocol.Dynamixel_Receive(data,len,MasterReceive)==Dynamixel_State_Success){

	ROS_INFO_STREAM("Ros Receive Success!");
	printf("DataList:\r\n");
						
	int nvalue;

	float fvalue;	
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive,  2, 2);
	printf("version:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 3, 2 + DisWheelXaxisReg - WheelRaduisReg);
	printf("radius of wheel:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 5, 6);
	printf("the distance by wheels on x axis:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 7, 8);
	printf("the distance by wheels on y axis:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 9, 9);
	printf("number of wheels:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 10, 11);
	myOdomData.linear_x = (float)nvalue/ 1000.0;
	printf("velocity on x axis:  %f \r\n", myOdomData.linear_x);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 12, 13);
	myOdomData.linear_y = (float)nvalue/ 1000.0;
	printf("velocity on y axis:  %f \r\n", myOdomData.linear_y);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 14, 15);
	myOdomData.linear_z = (float)nvalue/ 1000.0;
	printf("velocity on z axis:  %f \r\n", myOdomData.linear_z);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 16, 17);
	myOdomData.angular_x = (float)nvalue/ 1000.0;
	printf("Angular Velocity on x axis:  %f \r\n", myOdomData.angular_x);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 18, 19);
	myOdomData.angular_y = (float)nvalue/ 1000.0;
	printf("Angular Velocity on y axis:  %f \r\n", myOdomData.angular_y);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 20, 21);
	myOdomData.angular_z = (float)nvalue/ 1000.0;
	printf("Angular Velocity on z axis:  %f \r\n", myOdomData.angular_z);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 22, 25);
	myOdomData.position_x = (float)nvalue/ 1000.0;
	printf("Milemeter on x axis:  %f \r\n", myOdomData.position_x);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 26, 29);
	myOdomData.position_y = (float)nvalue/ 1000.0;
	printf("Milemeter on y axis:  %f \r\n", myOdomData.position_y);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 30, 33);
	myOdomData.position_z = (float)nvalue/ 1000.0;
	printf("Milemeter on z axis:  %f \r\n", myOdomData.position_z);

	
	//---
	float quaternion[4];
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 34, 35);
	quaternion[0] = (float)nvalue / 1000.0;//w
	

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 36, 37);
	quaternion[1] = (float)nvalue / 1000.0;//x
	

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 38, 39);
	quaternion[2] = (float)nvalue / 1000.0;//y
	

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 40, 41);
	quaternion[3] = (float)nvalue / 1000.0;//z
	

	myOdomData.orientation_x = quaternion[1];	
	myOdomData.orientation_y = quaternion[2];
	myOdomData.orientation_z = quaternion[3];
	myOdomData.orientation_w = quaternion[0];
	printf("orientation_x:  %f \r\n", quaternion[1]);
	printf("orientation_y:  %f \r\n", quaternion[2]);
	printf("orientation_z:  %f \r\n", quaternion[3]);
	printf("orientation_w:  %f \r\n", quaternion[0]);


	float fresuilt = atan2(2.f* (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]),
		quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3]);
	printf("cauculate resuilt of angle :  %f \r\n", fresuilt*180/3.14159);
	//----

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 42, 42);
	printf("Period time of sending:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 43, 44);
	printf("Ultrasonic sensor 1 value:  %d \r\n", nvalue);
	myRange = (float)nvalue/1000.0;//单位要求为m，底层上传单位是mm

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 45, 46);
	printf("Ultrasonic sensor 2 value:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 47, 48);
	printf("Ultrasonic sensor 3 value:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 49, 50);
	printf("Ultrasonic sensor 4 value:  %d \r\n", nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 51, 52);		
	printf("the IMU:  %f \r\n", (float)nvalue / 32768 * 180);
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 53, 54);
	printf("the IMU:  %f \r\n", (float)nvalue / 32768 * 180);	
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 55, 56);
	printf("the IMU:  %f \r\n", (float)nvalue / 32768 * 180);
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 57, 58);
	printf("the IMU:  %f \r\n", (float)nvalue / 32768 * 2000);
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 59 ,60);
	printf("the IMU:  %f \r\n", (float)nvalue / 32768 * 2000);
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 61, 62);
	printf("the IMU:  %f \r\n", (float)nvalue / 32768 * 2000);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 63, 64);
	float x = (float)nvalue;
	printf("the IMU:  %f \r\n", (float)nvalue);
	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 65, 66);
	float y = (float)nvalue;
	printf("the IMU:  %f \r\n", (float)nvalue);

	nvalue = protocol.Dynamixel_Value_Forward(MasterReceive, 67, 68);
	printf("the IMU:  %f \r\n", (float)nvalue);

	float alfa = atan2(y,x)*180/pi;
	printf("the IMU angle(not in IMU):  %f \r\n", alfa);

    }else{
        ROS_INFO("recv data error\n");
	
    }
   
	delete data;
	return Dynamixel_State_Success;
}
