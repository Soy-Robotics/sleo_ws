/**
 * @file /sleo_screen_driver/src/serial.cpp
 *
 * @brief Serial data get from Sleo.
 *
 * This package just read data from Sleo.
 *
 * @author Carl
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include "sleo_screen_driver/serial.hpp" 

namespace sleo_screen_driver{

	void SleoSerial::UART0_Close(int fd){  
	  close(fd);  
	}

	int SleoSerial::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity){  
     
      int   i;  
      int   status;  
      int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};  
      int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};  
           
      struct termios options;  
     
      /**tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1. 
    */  
      if  ( tcgetattr( fd,&options)  !=  0){  
        perror("SetupSerial 1");      
        return(FALSE);   
      }  
    
      //设置串口输入波特率和输出波特率  
      for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++){  
        if  (speed == name_arr[i]){               
          cfsetispeed(&options, speed_arr[i]);   
          cfsetospeed(&options, speed_arr[i]);    
        }  
      }       
     
      //修改控制模式，保证程序不会占用串口  
      options.c_cflag |= CLOCAL;  
      //修改控制模式，使得能够从串口中读取输入数据  
      options.c_cflag |= CREAD;  

      //设置数据流控制  
      switch(flow_ctrl){  
        case 0 ://不使用流控制  
               options.c_cflag &= ~CRTSCTS;  
               break;     
        case 1 ://使用硬件流控制  
               options.c_cflag |= CRTSCTS;  
               break;  
        case 2 ://使用软件流控制  
               options.c_cflag |= IXON | IXOFF | IXANY;  
               break;  
      }  
      //设置数据位  
      //屏蔽其他标志位  
      options.c_cflag &= ~CSIZE;  
      switch (databits){    
        case 5 :  
               options.c_cflag |= CS5;  
               break;  
        case 6 :  
               options.c_cflag |= CS6;  
               break;  
        case 7 :      
               options.c_cflag |= CS7;  
               break;  
        case 8 :      
               options.c_cflag |= CS8;  
               break;    
        default:     
                 fprintf(stderr,"Unsupported data size\n");  
                 return (FALSE);   
      }  
    
      //设置校验位  
      switch (parity){    
        case 'n':  
        case 'N': //无奇偶校验位。  
                options.c_cflag &= ~PARENB;   
                options.c_iflag &= ~INPCK;      
                break;   
        case 'o':    
        case 'O'://设置为奇校验      
                options.c_cflag |= (PARODD | PARENB);   
                options.c_iflag |= INPCK;               
                break;   
        case 'e':   
        case 'E'://设置为偶校验    
                options.c_cflag |= PARENB;         
                options.c_cflag &= ~PARODD;         
                options.c_iflag |= INPCK;        
                break;  
       case 's':  
       case 'S': //设置为空格   
                options.c_cflag &= ~PARENB;  
                options.c_cflag &= ~CSTOPB;  
                break;   
       default :    
                fprintf(stderr,"Unsupported parity\n");      
                return (FALSE);   
      }   
    
      // 设置停止位   
      switch (stopbits){    
        case 1:     
               options.c_cflag &= ~CSTOPB; break;   
        case 2:     
               options.c_cflag |= CSTOPB; break;  
        default:     
               fprintf(stderr,"Unsupported stop bits\n");   
               return (FALSE);  
      }  
     
    //修改输出模式，原始数据输出  
    options.c_oflag &= ~OPOST;  
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
    //options.c_lflag &= ~(ISIG | ICANON);  
     
    //设置等待时间和最小接收字符  
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */  
     
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
    tcflush(fd,TCIFLUSH);  
     
    //激活配置 (将修改后的termios数据设置到串口中）  
    if (tcsetattr(fd,TCSANOW,&options) != 0)    
           {  
               perror("com set error!\n");    
              return (FALSE);   
           }  
    return (TRUE);   
    }  

	int SleoSerial::UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity){  
	  int err;  
	  //设置串口数据帧格式  
  	  if (UART0_Set(fd,speed,flow_ctrl,databits,stopbits,'N') == FALSE){                                                           
        return FALSE;  
       }  
      else{  
        return  TRUE;  
      }  
	}  

	int SleoSerial::UART0_Recv(int fd, char *rcv_buf,int data_len){  
      int len,fs_sel;  
      fd_set fs_read;  
     
      struct timeval time;  
     
      FD_ZERO(&fs_read);  
      FD_SET(fd,&fs_read);  
     
      time.tv_sec = 10;  
      time.tv_usec = 0;  
     
      //使用select实现串口的多路通信  
      fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);  
      if(fs_sel){  
        len = read(fd,rcv_buf,data_len);   
        return len;  
       }  
      else{  
        fprintf(stderr,"Sorry,I am wrong! Can not read data from serial.\n");  
        return FALSE;  
       }       
	}  
 
	int SleoSerial::UART0_Send(int fd, char *send_buf,int data_len){  
      int len = 0;  
     
      len = write(fd,send_buf,data_len);  
      if (len == data_len ){  
        return len;  
      }       
      else{  
  		tcflush(fd,TCOFLUSH);  
        return FALSE;  
      }  
     
	}  
  
    int SleoSerial::UART0_Send(int fd,const char *w_buf,size_t len){
      ssize_t cnt = 0;

      cnt = write(fd,w_buf,len);
      if(cnt == -1){
        fprintf(stderr,"write error!\n");
        return -1;
      }

      return cnt;
    }

} // namespace sleo_screen_driver

