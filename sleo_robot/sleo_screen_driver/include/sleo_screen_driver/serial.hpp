/**
 * @file /sleo_screen_driver/include/sleo_screen_driver/serial.hpp
 *
 * @brief Serial data get from Sleo.
 *
 * This package just read data from Sleo.
 *
 * @author Carl
 *
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SLEO_SERIAL_
#define SLEO_SERIAL_
/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <stdio.h>      /*标准输入输出定义*/  
#include <stdlib.h>     /*标准函数库定义*/  
#include <unistd.h>     /*Unix 标准函数定义*/  
#include <sys/types.h>   
#include <sys/stat.h>     
#include <fcntl.h>      /*文件控制定义*/  
#include <termios.h>    /*PPSIX 终端控制定义*/  
#include <errno.h>      /*错误号定义*/  
#include <string.h>  
 
#include <fstream>
#include <iostream>  
#include <sstream>
#include <time.h>   
//宏定义  
#define FALSE  -1  
#define TRUE   0  
namespace sleo_screen_driver{
using namespace std;
/**
 * @ brief Driver of sleo for reading data.
 *
 * The SleoSerial keeps track of communication with motor driver with sleo.
 *
 */
class SleoSerial{
public:
	void UART0_Close(int fd);

	int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);

    /******************************************************************* 
	* 名称：                UART0_Init() 
	* 功能：                串口初始化 
	* 入口参数：         fd        :  文件描述符    
	*                  speed      :  串口速度 
	*                  flow_ctrl  :  数据流控制 
	*                  databits   :  数据位   取值为 7 或者8 
	*                  stopbits   :  停止位   取值为 1 或者2 
	*                  parity     :  效验类型 取值为N,E,O,,S 
	*                       
	* 出口参数：        正确返回为1，错误返回为0 
	*******************************************************************/  
	int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);

	/******************************************************************* 
	* 名称：                  UART0_Recv 
	* 功能：                接收串口数据 
	* 入口参数：        fd          :文件描述符     
	*                  rcv_buf     :接收串口中数据存入rcv_buf缓冲区中 
	*                  data_len    :一帧数据的长度 
	* 出口参数：        正确返回为1，错误返回为0 
	*******************************************************************/  
	int UART0_Recv(int fd, char *rcv_buf,int data_len);

	/******************************************************************** 
	* 名称：                  UART0_Send 
	* 功能：                发送数据 
	* 入口参数：        fd          :文件描述符     
	*                  send_buf    :存放串口发送数据 
	*                  data_len    :一帧数据的个数 
	* 出口参数：        正确返回为1，错误返回为0 
	*******************************************************************/  
	int UART0_Send(int fd, char *send_buf,int data_len);

	/******************************************************************** 
	* 名称：                  UART0_Send 
	* 功能：   发送数据 
	* 入口参数：        fd          :文件描述符     
	*                  send_buf    :存放串口发送数据 
	*                  data_len    :一帧数据的个数 
	* 出口参数：        正确返回为1，错误返回为0 
	*******************************************************************/  
    int UART0_Send(int fd,const char *w_buf,size_t len);
};

} // namespace sleo_screen_driver

#endif /* SLEO_SERIAL_ */
