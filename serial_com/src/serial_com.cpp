#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

#include </home/linux/work/catkin_ws/devel/include/key_read/keyValue.h> 

serial::Serial ser; //声明串口对象 


//回调函数 
void write_callback(const key_read::keyValue::ConstPtr& msg) 
{ 
    if(ser.isOpen()) 
    { 
        ser.write(msg->value.c_str());   //发送串口数据 
        ROS_INFO_STREAM("Writing to serial port:" << msg->value.data() );     
    } 
    else
    {
        ROS_INFO_STREAM("Serial Port is close");
    }
    
} 

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_com"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    //订阅主题，并配置回调函数 
    ros::Subscriber write_sub = nh.subscribe("key_info", 2, write_callback); 
    // ros::spin(); 
    //发布主题 
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(1000); 
    while(ros::ok()) 
    { 

        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << result.data); 
            // read_pub.publish(result); 
            // ser.write(result.data);   //发送串口数据 
        } 

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spin(); 
        loop_rate.sleep(); 
    } 
}

