using namespace std;
#include <fstream>
#include <math.h>
#include <cmath>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
//topic
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

//OpenCV头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include<image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息
class bebopControl
{
public:
    bebopControl();

    ros::NodeHandle nh;
    image_transport::ImageTransport *it;
    geometry_msgs::Twist cmd_vel;
    
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher cmd_pub;

    ros::Subscriber state_sub;
    image_transport::Subscriber image_sub;

    cv::Mat image ;
    bool recImageFlag;

    ros::Rate *rate;
    ros::Time start_time;
    void takeoff();
    void land();
    void cmd(float x,float y,float z);
    void state_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void image_cb(const sensor_msgs::ImageConstPtr& msg);
    void start();

private:
    /* data */
};


bebopControl::bebopControl()
{
    ROS_INFO("bebop  init! ");
    rate = new ros::Rate(20.0);

    it = new image_transport::ImageTransport(nh);

    takeoff_pub= nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);         // 发布 起飞命令
    land_pub   = nh.advertise<std_msgs::Empty>("bebop/land", 1);            // 发布 降落命令
    cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);    // 发布 移动命令
    state_sub = nh.subscribe<nav_msgs::Odometry>("bebop/odom", 5, &bebopControl::state_cb,this);// 订阅 Bebop当前状态 
    image_sub = it->subscribe("bebop4/image_raw",1,&bebopControl::image_cb,this);

    recImageFlag = false;
}

void bebopControl::takeoff()
{
    takeoff_pub.publish(std_msgs::Empty());
    ROS_INFO("bebop  takeoff! ");
}
void bebopControl::land()
{
    land_pub.publish(std_msgs::Empty());
    ROS_INFO("bebop  land! ");
}
void bebopControl::cmd(float x,float y,float z)
{
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.linear.z = z;
    cmd_pub.publish(cmd_vel);
    ROS_INFO("bebop  send cmd! ");
}
void bebopControl::state_cb(const nav_msgs::Odometry::ConstPtr& msg)
{

}
 void bebopControl::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image = cv_bridge::toCvShare(msg,"bgr8")->image;
        recImageFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}
void bebopControl::start()
{
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);
    //bebop
    // takeoff_pub= nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);         // 发布 起飞命令
    // land_pub   = nh.advertise<std_msgs::Empty>("bebop/land", 1);            // 发布 降落命令
    // cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);    // 发布 移动命令
    // state_sub = nh.subscribe<nav_msgs::Odometry>("bebop/odom", 5, &bebopControl::state_cb,this);// 订阅 Bebop当前状态 
    // image_sub = it.subscribe("bebop4/image_raw",1,image_cb);
    // ros::Duration(3).sleep();//必须延时一段时间，不然飞不起来
    // this->takeoff();        //起飞
    // ros::Duration(5).sleep();
    // this->cmd(1,0,0);       //向前飞（机头下沉1弧度，此处只有一个动作）
    // ros::Duration(3).sleep();
    // this->cmd(0,0,-0.6);
    // ros::Duration(5).sleep();
    // this->land();
    // this->cmd(0,0,0.1);
    // ros::Duration(3).sleep();

    while(ros::ok())
    {
        /*以0.1m/s的速度下降*/
        // this->cmd(0,0,-0.1);

        if (recImageFlag == true)
        {
            recImageFlag = false;
            cv::Mat img = image;
            cv::imshow("image",img);
            cv::waitKey(5);

        }

        ros::spinOnce();
        rate->sleep();
    }

}
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "bebop_control");
    
    bebopControl bebopTest;
    bebopTest.start();

    return 0;
}

