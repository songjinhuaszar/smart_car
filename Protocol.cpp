#include "SerialClass.hpp"
#include "protocol.h"

unsigned int Protocol::Dynamixel_Receive(unsigned char *Data,unsigned char Data_Length, unsigned char *Return_Data)//ID Address Value
{
   	unsigned char  i;
	unsigned char Data2[1];
	unsigned int Return_State = Dynamixel_State_Error;
	for (i = 0; i<Data_Length; i++)
	{
		if (Blk_Num<2)
		{
			if ((unsigned char)Data[i] == Dynamixel_Header[Blk_Num])
			{
				Data_Blk[Blk_Num] = (unsigned char)Data[i];
				Blk_Num++;
			}
			else Blk_Num = 0;
		}

		else if (Blk_Num == 2)
		{
			if ((unsigned char)Data[i] == Dynamixel_Reserved)
			{
				Data_Blk[Blk_Num] = (unsigned char)Data[i];
				Blk_Num++;
			}
			else Blk_Num = 0;
		}

		else if (Blk_Num ==3) //Packet Length
		{

			Data_Blk[Blk_Num] = (unsigned char)Data[i];
			Blk_Num++;
		}

		else if (Blk_Num == 4)//Instruction
		{
			if (((unsigned char)Data[i] == Dynamixel_Instruction_Return) || ((unsigned char)Data[i] == Dynamixel_Instruction_Error) ||
				((unsigned char)Data[i] == Dynamixel_Instruction_Read) || ((unsigned char)Data[i] == Dynamixel_Instruction_Write))
			{
				Data_Blk[Blk_Num] = (unsigned char)Data[i];
				Blk_Num++;
			}
			
			else Blk_Num = 0;
		}

		else if (Blk_Num ==5)//Payload
		{
			Data_Blk[Blk_Num] = (unsigned char)Data[i];
			Blk_Num++;

			if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Error)
			{
				Data_Blk[Blk_Num] = (unsigned char)Data[i];

				Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Data_Blk,Data_Blk[3] + 2), 1, Data2);

				//printf("test:0x%X ", Data2[0]);//test
				//printf("test:0x%X ", Data_Blk[Data_Blk[3] + 3]);//test

				if (Data2[0] == Data_Blk[Data_Blk[3] + 3])
				{
					Return_State = Dynamixel_State_Success;
					Return_Data[0] = (unsigned char)Data_Blk[5];//Payload	

					Blk_Num = 0;
				//	printf("0x%X \n", Return_Data[0]);
				}
			}
			
		}
		
		else if (Blk_Num < Data_Blk[3]+4)
		{
			Data_Blk[Blk_Num] = (unsigned char)Data[i];
			Blk_Num++;
			
			if (Blk_Num == Data_Blk[3] + 4)
			{
				Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Data_Blk, Data_Blk[3] + 3), 1, Data2);
			

			//	printf("0x%X ", Data2[0]);//test
			//	printf("0x%X ", Data_Blk[Data_Blk[3] + 3]);//test

				if (Data2[0] == Data_Blk[Data_Blk[3] + 3])
				{
					Return_State = Dynamixel_State_Success;
					if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Error)
					{
						Return_Data[0] = (unsigned char)Data_Blk[5];//Payload	
					}
					else
					{
						Return_Data[0] = (unsigned char)Data_Blk[5];//Address
						Return_Data[1] = Data_Blk[3] - 3;
						for (char k = 0; k<Return_Data[1]; k++)
						{
							Return_Data[2 + k] = (unsigned char)Data_Blk[6 + k];
						
						}
						//ROS_INFO("receive data:\r\n");
					//	for (char k = 0; k<Return_Data[1]+2; k++)
						{
							
					//		ROS_INFO("0x%X ", Return_Data[k]);
						}
					}
				}
				Blk_Num = 0;
			}
		}
		else if (Blk_Num >= Uart_Blk_Length)
			Blk_Num = 0;
	}

	ROS_INFO("the num_blk is %d",Blk_Num);	
     return Return_State;
}

unsigned  char Protocol::Dynamixel_Send(unsigned char Instruction, unsigned short Address, unsigned int *Value, 
	unsigned char Value_Length, unsigned char *Return_Data) {
    
	unsigned char i = 0;
	unsigned char Data4[4];
	unsigned char Data[1];
	unsigned char Length = 0;

	//	Send_Data = (unsigned char*)malloc(sizeof(unsigned char) *(Value_Length+5+7));

	//Header
	for (i = 0; i<2; i++)
	{
		Return_Data[Length] = Dynamixel_Header[i];
		Length++;
	}

	//Reserved
	Return_Data[Length] = Dynamixel_Reserved;
	Length++;

	//Packet Length
	if (Instruction==0x02)
	{
		Return_Data[Length] = 0x04;
		Length++;
	}
	else//0x03
	{
		if (Address==0x09)
		{
			Return_Data[Length] = 12 + 3;
			Length++;
		}
		else
		{
			Return_Data[Length] = Value_Length + 3;
			Length++;
		}
		
	}

	//Instruction
	Return_Data[Length] = Instruction;
	Length++;

	//Start Address
	Return_Data[Length] = Address;
	Length++;

	//Data
	if (Instruction==0x02)
	{
		Return_Data[Length] = Value_Length;
		Length++;
	}
	else//0x03
	{
		if (Address==0x09)
		{
			for (int j = 0; j < 6;j++)
			{
				Dynamixel_Value_Dackward(Value[j], 2, Data4);
				for (i = 0; i<2; i++)
				{
					Return_Data[Length] = Data4[i];
					Length++;
				}
			}

		}else
			{
				Dynamixel_Value_Dackward(Value[0], Value_Length, Data4);
				for (i = 0; i<Value_Length; i++)
				{
					Return_Data[Length] = Data4[i];
					Length++;
				}
			}
	}

	//CRC
	Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Return_Data, Length), 1, Data);

	Return_Data[Length] = Data[0];
	Length++;

	ROS_INFO_STREAM("the send data is ");
	for (int i = 0; i < Length; i++)
	{
		ROS_INFO("OX%02x ", Return_Data[i]);
	}

	return Length;

}

unsigned short Protocol::Dynamixel_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
                                              unsigned short data_blk_size)
{
    unsigned short i, j;
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ Dynamixel_crc_table[i];
    }

    return crc_accum;
}

void Protocol::Dynamixel_Value_Dackward(unsigned int Value, unsigned char data_blk_size, unsigned char  Return_data[])
{
    unsigned char i;
    for(i=0;i<data_blk_size;i++)
    {
        Return_data[i] = Value & 0xff;
        Value = Value>>8;
    }
}

 int Protocol::Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start,
                                       unsigned char data_blk_size_end)
{
	int Value = 0x00000000;
  unsigned char i;
  //if(data_blk_size_start == data_blk_size_end)
  //return Dynamixel_Return_Success;
  for (i = data_blk_size_end; i >= data_blk_size_start; i--)
  {
    Value = Value << 8;
    Value = Value | data_blk_ptr[i];
  }
  if(data_blk_size_end-data_blk_size_start<2)
  {
    if (Value > Dynamixel_Return_Position_Value_Max )
      Value -= (0xffff + 1);
  }
  else  if (Value > (0xffffffff/2-1000))
      Value -= (0xffffffff + 1);
  return Value;	

}
