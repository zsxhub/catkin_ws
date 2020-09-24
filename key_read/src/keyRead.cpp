#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  

#include <ros/ros.h>  
#include <key_read/keyValue.h>  
 

int kfd = 0;  
struct termios termios_old, raw;  
bool isok = true;  
 

int main(int argc, char** argv)  
{  

    ros::init(argc, argv,"key_read");
    ros::NodeHandle nh;//创建句柄
    key_read::keyValue keyValueMsg;

    ros::Publisher pub = nh.advertise<key_read::keyValue>("key_info",1);
    tcgetattr(kfd, &termios_old); 
    memcpy(&raw, &termios_old, sizeof(struct termios)); 
    //使用标准输入模式  显示输入字符
    raw.c_lflag &=~ (ICANON | ECHO);  
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    tcsetattr(kfd, TCSANOW, &raw); 
    ROS_INFO("Please input keys, press Ctrl + C to quit'");
/**************************************

    int poll(struct pollfd *fds, nfds_t nfds, int timeout);
    功能：监视并等待多个文件描述符的属性变化 
    参数：
        fds：指向一个结构体数组的第0个元素的指针
        nfds：表示fds结构体数组的长度 
        timeout：表示poll函数的超时时间，单位是毫秒 
**************************************/
    struct pollfd ufd;  
    ufd.fd = kfd;  
    ufd.events = POLLIN;  // POLLIN   数据可读
    char c; 

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        /* code for loop body */

        // get the next event from the keyboard  
        int num;  
        num = poll(&ufd, 1, 250);

        if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {   
                ROS_INFO("read() fiel");
                continue;  
            }
            else
            {
                switch(c)  
                {  
                    /**********小车前进************/
                    case 'w':  
                    keyValueMsg.value = "w";
                    ROS_INFO("KEYCODE_W");
                    break;
                    /**********小车后退************/
                    case 's':  
                    ROS_INFO("KEYCODE_S");
                    keyValueMsg.value = "s";
                    break; 
                    /**********小车左转************/                             
                    case 'a':  
                    ROS_INFO("KEYCODE_A");
                    keyValueMsg.value = "a";
                    break;
                    /**********小车右转************/                   
                    case 'd':  
                    ROS_INFO("KEYCODE_D");
                    keyValueMsg.value = "d";
                    break; 
                    /**********1号舵机************/
                    case 'y':  
                    keyValueMsg.value = "y";
                    ROS_INFO("KEYCODE_y");
                    break;
                    case 'u':  
                    keyValueMsg.value = "u";
                    ROS_INFO("KEYCODE_u");
                    break;
                    /**********2号舵机************/
                    case 'i':  
                    keyValueMsg.value = "i";
                    ROS_INFO("KEYCODE_i");
                    break;
                    case 'o':  
                    keyValueMsg.value = "o";
                    ROS_INFO("KEYCODE_o");
                    break; 
                    /**********3号舵机************/
                    case 'h':  
                    keyValueMsg.value = "h";
                    ROS_INFO("KEYCODE_h");
                    break;
                    case 'j':
                    keyValueMsg.value = "j";  
                    ROS_INFO("KEYCODE_j");
                    break;
                    /**********4号舵机************/
                    case 'k':  
                    keyValueMsg.value = "k";
                    ROS_INFO("KEYCODE_k");
                    break;
                    case 'l':  
                    keyValueMsg.value = "l";
                    ROS_INFO("KEYCODE_l");
                    break; 
                    /**********5号舵机************/
                    case 'n':  
                    keyValueMsg.value = "n";
                    ROS_INFO("KEYCODE_n");
                    break;
                    case 'm':  
                    keyValueMsg.value = "m";
                    ROS_INFO("KEYCODE_m");
                    break;
                    /**********6号舵机************/
                    case ',':  
                    keyValueMsg.value = ",";
                    ROS_INFO("KEYCODE_,");
                    break;
                    case '.':  
                    keyValueMsg.value = ".";
                    ROS_INFO("KEYCODE_.");
                    break;    

                    case 'v':  
                    keyValueMsg.value = "v";
                    ROS_INFO("KEYCODE_v");
                    break; 

                    default:
                    isok = false;
                    break;
                } 
                // 发布按键消息
                if(isok == true)
                    pub.publish(keyValueMsg); 
                else
                 {
                     isok = true;
                     ROS_INFO("invalid character !!!");
                 }   
            }  
        }  
        loop_rate.sleep();
    }
    //恢复终端设置
    tcsetattr(kfd, TCSANOW, &termios_old);   
    return 0;

}  
