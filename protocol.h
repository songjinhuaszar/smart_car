//
// Written by songjinhua 2019-3-19
//

#ifndef HANDCONTROL_PROTOCOL_H
#define HANDCONTROL_PROTOCOL_H

#define Uart_Blk_Length 100
#define pi 3.14159
#define Dynamixel_Reserved 0x00

/*********Instruction************/
#define Dynamixel_Instruction_Read 0x02
#define Dynamixel_Instruction_Write 0x03
#define Dynamixel_Instruction_Return 0x55
#define Dynamixel_Instruction_Error 0x66


#define Dynamixel_State_Success      0
#define Dynamixel_State_Incomplete   0x01
#define Dynamixel_State_overrange    0x02
#define Dynamixel_State_Error        0x03

#define Dynamixel_Return_Position_Value_Max 15000
#define Dynamixel_Return_Success 150001
#define Dynamixel_Return_Error   150002

//--------address
#define VersionReg          1   //version---2
#define WheelRaduisReg      2   //radius of wheel--3
#define DisWheelXaxisReg      4  //the distance by wheels on x axis--3+2=5
#define DisWheelYaxisReg      6   //the distance by wheels on y axis---5+6-4=7
#define NumWheelReg      8   //number of wheels---7+8-6=9
#define XaxisVelocityReg      9   //velocity on x axis----9+9-8=10
#define YaxisVelocityReg      11   //velocity on y axis---10+11-9=12
#define ZaxisVelocityReg      13   //velocity on z axis--12+13-11=14
#define XaxisAngularVelocityReg      15   //Angular Velocity on x axis--14+15-13=16
#define YaxisAngularVelocityReg      17   //Angular Velocity on y axis--16+17-15=18
#define ZaxisAngularVelocityReg      19   //Angular Velocity on z axis--18+19-17=20
//----20190327
#define XaxisMilemeterReg      21   //Milemeter on x axis--20+21-19=22
#define YaxisMilemeterReg      25   //Milemeter on y axis--22+25-21 = 26
#define ZaxisMilemeterReg      29   //Milemeter on z axis--26+29-25=30

#define quaternion0   33   //30+33-29=34
#define quaternion1   35   //34+35-33=36
#define quaternion2   37   //36+37-35=38
#define quaternion3   39   //38+39-37=40
//-----
#define SendFrequencyReg      41   //Period time of sending--40+41-39=42
#define Ultrasonic1Reg      42   //Ultrasonic sensor 1 value--42+42-41=43
#define Ultrasonic2Reg      44   //Ultrasonic sensor 2 value--43+44-42=45
#define Ultrasonic3Reg      46   //Ultrasonic sensor 3 value--45+46-44=47
#define Ultrasonic4Reg      48   //Ultrasonic sensor 4 value--47+48-46=49
#define IMUReg      50 //Pose information--49+50-48=51
#define InfraredReg      68   //Infrared sensor value--51+68-50=69
#define ManipulatorReg      70   //Manipulator Matrix--69+70-68=71
#define MaxDataBuffer 80
//----------

class Protocol{
public:
    int Blk_Num =0;
	
    unsigned char *Data_Blk;

    const unsigned char Dynamixel_Header[2] = {
    0xaa, 0xbb
};

   

    const unsigned short Dynamixel_crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
     int Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start, unsigned char data_blk_size_end);
    void  Dynamixel_Value_Dackward(unsigned int Value , unsigned char data_blk_size , unsigned char *Return_data);
    unsigned short Dynamixel_update_crc(unsigned short crc_accum,unsigned char *data_blk_ptr, unsigned short data_blk_size);
public:
    Protocol():Data_Blk(new unsigned char[Uart_Blk_Length]{0}){}


	unsigned int Dynamixel_Receive(unsigned char *Data,unsigned char Data_Length, unsigned char *Return_Data);//ID Address Value
	unsigned  char Dynamixel_Send(unsigned char Instruction, unsigned short Address, unsigned int *Value, 
	unsigned char Value_Length, unsigned char *Return_Data);
    
};

#endif //HANDCONTROL_PROTOCOL_H
