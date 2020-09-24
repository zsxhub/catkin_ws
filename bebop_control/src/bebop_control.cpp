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

class bebopControl
{
public:
    bebopControl();

    ros::NodeHandle nh;

    geometry_msgs::Twist cmd_vel;
    
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher cmd_pub;

    ros::Subscriber state_sub;

    ros::Rate *rate;
    ros::Time start_time;
    void takeoff();
    void land();
    void cmd(float x,float y,float z);
    void state_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void start();

private:
    /* data */
};


bebopControl::bebopControl()
{
    ROS_INFO("bebop  init! ");
    rate = new ros::Rate(10.0);
    //bebop
    takeoff_pub= nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);         // 发布 起飞命令
    land_pub   = nh.advertise<std_msgs::Empty>("bebop/land", 1);            // 发布 降落命令
    cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);    // 发布 移动命令
    state_sub = nh.subscribe<nav_msgs::Odometry>("bebop/odom", 5, &bebopControl::state_cb,this);// 订阅 Bebop当前状态 
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
void bebopControl::start()
{
    
    ros::Duration(3).sleep();//必须延时一段时间，不然飞不起来
    this->takeoff();        //起飞
    ros::Duration(5).sleep();
    this->cmd(1,0,0);       //向前飞（机头下沉1度，此处只有一个动作）
    ros::Duration(3).sleep();
    // this->cmd(0,0,-0.6);
    // ros::Duration(5).sleep();
    // this->land();
    // this->cmd(0,0,0.1);
    // ros::Duration(3).sleep();

    while(ros::ok())
    {
        /*以0.1m/s的速度下降*/
        this->cmd(0,0,-0.1);
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

