#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "smart_car/smart_car_cmd.h"
#include <string>
#include "protocol.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/Range.h>

using namespace serial;
using namespace std;


class SerialClass {

  //  const string Version = "ReadV";
public:
    struct OdomData
{
    float position_x=0.0;
    float position_y=0.0;
    float position_z=0.0;
    float orientation_x=0.0;
    float orientation_y=0.0;
    float orientation_z=0.0;
    float orientation_w=1.0;

    float linear_x=0.0;
    float linear_y=0.0;
    float linear_z=0.0;
    float angular_x=0.0;
    float angular_y=0.0;
    float angular_z=0.0;
}myOdomData;

public:
    float myRange;


private:
    
    string ttyName;
    int Baudrate;
    int timeout;
    int value;
unsigned char MasterReceive[100]={0};
    
    unsigned char *data;
 
    unsigned char  Address;

	Protocol protocol;

public:
 
	Serial ser;

	static void callback(const smart_car::smart_car_cmd::ConstPtr msg);

	
	static void callback_geometry(const geometry_msgs::Twist & cmd_input);


	unsigned char SendSpeed(unsigned int *Value);

    SerialClass(string ttyName, int Baudrate, int timeout = 1000);

    unsigned char sendCmd(const smart_car::smart_car_cmd::ConstPtr msg=NULL);

    unsigned char recvData(size_t len);

    void run();


};
