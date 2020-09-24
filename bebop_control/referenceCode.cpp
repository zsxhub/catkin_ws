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

std_msgs::Empty takeoff;
std_msgs::Empty land;
geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry bp1_odom;
geometry_msgs::PoseStamped bp1_vrpn_odom;
//Desired Position
Eigen::Vector2d set_xy;
double t;

double Kr1,Kv1;
double delta_ux,delta_uy,delta_x,delta_y;

double z,yaw;
double current_x,current_y,last_x,last_y;
double current_x_v,current_y_v,current_x_a,current_y_a;
double delta_u_x,delta_u_y,delta_v_x,delta_v_y;
double delta_rotation;

int flag_ahead;
int Flag_land;

int i,j;
Eigen::Vector2d xy;

Eigen::Vector3d bp1_last_position,bp1_current_position,bp1_current_v,bp1_last_v,bp1_current_a;
Eigen::Vector3d bp1_current_euler,bp1_last_euler;
Eigen::Matrix2d Rotation_R2B;
double theta;

//PID
double Kp_rou = 0.5,   Ki_rou = 0,      Kd_rou = 0.32;       //line.x   roll
double Kp_yaw = 0.5,   Ki_yaw = 0,      Kd_yaw = 0.32;       //line.y   pitch
double Kp_gaz = 0.6,   Ki_gaz = 0.01,   Kd_gaz = 0.2;       //line.z   vertical velocity?
float Kp_theta = 0.05, Ki_theta = 0,   Kd_theta = 0;     //angular.z   yaw rotational speed
//Max Current
double max_tilt_angle=0.1;   //  line.x, line.y
double max_vert_speed=1;   //  line.z
double max_rot_speed=10;    //  angular.z
double z_kp,z_kd;
double dyaw;
static ros::Time start_time;

double pi=3.14159265;
double delta_t= 30;

ros::Publisher bp1_land_pub;

void printf_state();//打印函数
Eigen::Vector3d setDronePosition(const geometry_msgs::Point pose_drone);
Eigen::Vector3d QuaterniondToEuler(const geometry_msgs::Quaternion quaternion_drone);
double current_time();

void bp1_odom_sub(const nav_msgs::Odometry::ConstPtr& msg)
{    bp1_odom = *msg;
}

void bp1_vrpn(const geometry_msgs::PoseStamped::ConstPtr& msg1)
{    bp1_vrpn_odom = *msg1;
}

void land_flag_callback(const std_msgs::Int32::ConstPtr& msg2)
{    Flag_land = msg2->data;
     if (Flag_land == 233)
     {
         flag_ahead = 0;
         bp1_land_pub.publish(std_msgs::Empty());
     }
}


int main(int argc, char **argv)
{   ros::init(argc, argv, "ardronetest_control1");
    ros::NodeHandle nh;

    //bebop1 192.168.1.110  bebop1 4黑
    ros::Publisher bp1_takeoff_pub= nh.advertise<std_msgs::Empty>("bebop1/takeoff", 1);         // 发布 起飞命令
    bp1_land_pub   = nh.advertise<std_msgs::Empty>("bebop1/land", 1);            // 发布 降落命令
    ros::Publisher bp1_cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop1/cmd_vel", 1);    // 发布 移动命令
    ros::Subscriber bp1_state_vrpn = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/bebop1/pose", 5, bp1_vrpn);
    ros::Subscriber bp1_state_sub = nh.subscribe<nav_msgs::Odometry>("bebop1/odom", 5, bp1_odom_sub);// 订阅 Bebop当前状态

    ros::Subscriber land_sub = nh.subscribe<std_msgs::Int32>("land_flag", 5, land_flag_callback);// 订阅 Bebop当前状态

    //ros::Subscriber drop_flag_Sub = nh.subscribe<std_msgs::UInt16>("/drop_flag", 10, drop_flag_callback);

    ros::Rate rate(30);// 状态更新频率 [50Hz]
    //ros::Rate rate(2);    //ros::Rate loop_rate(50);

    ros::Duration(3).sleep();
    bp1_takeoff_pub.publish(std_msgs::Empty());   //起飞
    ros::Duration(3).sleep();

    ros::spinOnce();
    printf_state();
    rate.sleep();

    //cin >> flag_ahead;
    flag_ahead=1;

    i = 0;
    yaw = 0;
    z = 1.4; //设高度恒为1.2

    Kr1 = 1.5;   //1
    Kv1 = 2.5; //2 一致性算法中的变量

    bp1_current_position = setDronePosition(bp1_vrpn_odom.pose.position);
    bp1_current_v << 0, 0, 0;

    start_time = ros::Time::now();

    while(flag_ahead)
    { 
        t = current_time();                   //获取当前时间
        //参考基准状态 位置　速度　加速度

        /*current_x=0.6*cos(t/6*2*pi);         //参考基准状态 current_x
      current_y=0.6*sin(t/6*2*pi);         //参考基准状态 current_y
      current_x_v=-0.6*sin(t/6*2*pi)*2*pi/6;         //参考基准状态导数 速度 current_x_v
      current_y_v=0.6*cos(t/6*2*pi)*2*pi/6;          //参考基准状态导数 速度 current_y_v
      current_x_a=-0.6*cos(t/6*2*pi)*2*pi/6*2*pi/6;         //参考基准状态导数的导数 加速度 current_x_a
      current_y_a=-0.6*sin(t/6*2*pi)*2*pi/6*2*pi/6;          //参考基准状态导数的导数 加速度 current_y_a*/


        if ( t < 6 + 12 * 0 )  //飞到起点
        { current_x = 1.2 - 0.5;
            current_x_v = 0;
            current_x_a = 0;

            current_y = -1.2;
            current_y_v = 0;
            current_y_a = 0;
        }
        else if (t< 6 + 12 * 1) // y往前飞
        {current_x = 1.2 - 0.5;
            current_x_v = 0;
            current_x_a = 0;

            current_y = -1.2 + 0.2 * (t - 6 - 12 * 0);
            current_y_v = 0.2;
            current_y_a = 0;
        }
        else if (t< 6 + 12 * 2) // x往后飞
        {current_x = 1.2 - 0.2 * (t - 6 - 12 * 1) - 0.5;
            current_x_v = -0.2;
            current_x_a = 0;

            current_y = 1.2;
            current_y_v = 0;
            current_y_a = 0;
        }
        else if (t< 6 + 12 * 3) // y往后飞
        {current_x = -1.2 - 0.5;
            current_x_v = 0;
            current_x_a = 0;

            current_y = 1.2 - 0.2*(t - 6 - 12 * 2);
            current_y_v = -0.2;
            current_y_a = 0;
        }
        else if (t< 6 + 12 * 3 + 6) // 静止
        {current_x = -1.2 - 0.5;
            current_x_v = 0;
            current_x_a = 0;

            current_y = -1.2;
            current_y_v = 0;
            current_y_a = 0;
        }

        /*
      if (t<6)  //飞到起点
      {current_y = -1.2;
       current_y_v = 0;
       current_y_a = 0;

       current_x = 1.2 - 0.5;
       current_x_v = 0;
       current_x_a = 0;
      }
      else if (t< 6 *2) // y往前飞
       {current_y = -1.2 + 0.4*(t - 6 *1);
        current_y_v = 0.4;
        current_y_a = 0;

        current_x = 1.2 - 0.5;
        current_x_v = 0;
        current_x_a = 0;
       }
      else if (t< 6 *3) // x往后飞
       {current_y = 1.2;
        current_y_v = 0;
        current_y_a = 0;

        current_x = 1.2 - 0.4*(t - 6 *2) - 0.5;
        current_x_v = -0.4;
        current_x_a = 0;
       }
      else if (t< 6 *4) // y往后飞
       {current_y = 1.2 - 0.4*(t - 6 *3);
        current_y_v = -0.4;
        current_y_a = 0;

        current_x = -1.2 - 0.5;
        current_x_v = 0;
        current_x_a = 0;
       }
      else if (t< 6 *5) // x往前飞
       {current_x = -1.2 + 0.4*(t - 6 *4) - 0.5;
        current_x_v = 0.4;
        current_x_a = 0;

        current_y = -1.2;
        current_y_v = 0;
        current_y_a = 0;
       }
      else if (t< 6 *6) //禁止
       {current_x = 1.2 - 0.5;
        current_x_v = 0;
        current_x_a = 0;

        current_y = -1.2;
        current_y_v = 0;
        current_y_a = 0;
       }*/

        /*   此部分用作演示用
      if (t<6)  //飞到起点
      {current_y = -2;
       current_y_v = 0;
       current_y_a = 0;
      }
      else if (t<16) //y往前飞
       {current_y = -2 + 0.4*(t-6);
        current_y_v = 0.4;
        current_y_a = 0;
       }
      else if (t<20) //y前飞后停止
       {current_y = 2;
        current_y_v = 0;
        current_y_a = 0;
       }
      else if (t<36) //y往后飞 ,同时 x向前
       {current_x = 0.2 + 0.1*(t-20);
        current_x_v = 0.1;
        current_y = 2 - 0.125*(t-20);
        current_y_v = -0.125;
        current_y_a = 0;
       }
      else if (t<40) //禁止
       {current_x = 1.8;
        current_x_v = 0;
        current_y = 0;
        current_y_v = 0;
        current_y_a = 0;
       }
      else if (t<50) //禁止
       {current_x = 1.8 - 0.3*(t-40);
        current_x_v = -0.3;
        current_y = 0;
        current_y_v = 0;
        current_y_a = 0;
       }
      else
       //if (t<50) //禁止
       {current_x = -1.2;
        current_x_v = 0;
        current_y = 0;
        current_y_v = 0;
        current_y_a = 0;
       }*/

        //上一时刻　bp1　位置　速度
        bp1_last_position = bp1_current_position;    //bp1上一时刻位置
        bp1_last_v = bp1_current_v;                  //bp1上一時刻速度
        //当前时刻　bp1　位置　角度　速度　加速度
        bp1_current_position = setDronePosition(bp1_vrpn_odom.pose.position);//bebop1 当前位置
        bp1_current_euler = QuaterniondToEuler(bp1_vrpn_odom.pose.orientation);//bebop1 当前姿态
        bp1_current_v = (bp1_current_position - bp1_last_position) * delta_t;  //bebop1 当前速度 delta_t 为 30 故用乘法
        bp1_current_a = (bp1_current_v - bp1_last_v) * delta_t;       //bebop1 当前加速度 delta_t 为 30  时间间隔为1/30 故用乘法

        //if(1+1<01)
        if ( (bp1_current_position[0]>2.2)| (bp1_current_position[1]>3.2)| (bp1_current_position[1]<-3) |((bp1_current_position[2])>2.5) | (bp1_current_position[0]<-3) )
        {bp1_land_pub.publish(std_msgs::Empty());
            flag_ahead = 0;
        } //判断bp1是否出界 2*2
        else
            if (t>= 12 * 4)   //t>=53
            { bp1_land_pub.publish(std_msgs::Empty());
                flag_ahead=0;
            } //时间设置,30s降落
            else
            { //根据一致性算法算出期望delta_ux,delta_ux
                //bp1唯一可以访问参考状态
                delta_u_x= current_x_a - Kr1*(bp1_current_position[0]-current_x) - Kv1*(bp1_current_v[0]-current_x_v); //指令u_x
                delta_u_y= current_y_a - Kr1*(bp1_current_position[1]-current_y) - Kv1*(bp1_current_v[1]-current_y_v); //指令u_x
                delta_rotation=bp1_current_euler[0];
                xy << delta_u_x,  delta_u_y;
                Rotation_R2B << cos(delta_rotation),sin(delta_rotation),-sin(delta_rotation),cos(delta_rotation);
                //line.x,line.y
                xy = Rotation_R2B*xy;

                printf_state();
                //line.z
                z_kp = Kp_gaz * (z-bp1_current_position[2]);
                z_kd = Kd_gaz * (bp1_current_position[2]-bp1_last_position[2]) * delta_t ;

                //angular.z
                dyaw = 0 - bp1_current_euler[0];
                if (dyaw > pi)
                    dyaw = dyaw - 2*pi;
                else
                    if (dyaw <= -pi)
                        dyaw = dyaw + 2*pi;
                dyaw = Kp_theta * dyaw;

                cmd_vel.linear.x =  xy[0] * max_tilt_angle;
                cmd_vel.linear.y =  xy[1] * max_tilt_angle;
                cmd_vel.linear.z =  (z_kp-z_kd) * max_vert_speed;
                cmd_vel.angular.z= dyaw * max_rot_speed;
                cout << "[linear. x, y]: " << xy[0] <<"  ,"<<xy[1]<< endl;
                cout << "[linear.z, yaw]:" << cmd_vel.linear.z<< ", "<<cmd_vel.angular.z<<endl;
                bp1_cmd_pub.publish(cmd_vel);
            }

        ofstream outfile;
        outfile.open("/home/zy/datetest/bp_1.txt",ios::app);
        outfile<<t<<", "<< current_x <<", "<<current_y<<", "<<z<<", ";
        outfile<<bp1_vrpn_odom.pose.position.x<<", "<<bp1_vrpn_odom.pose.position.y<<", ";
        outfile<<bp1_vrpn_odom.pose.position.z<<", "<<bp1_current_euler[0]*180/pi<<", ";
        outfile<<cmd_vel.linear.x<<", "<<cmd_vel.linear.y<<", "<<cmd_vel.linear.z<<", "<<cmd_vel.angular.z<<endl;
        outfile.close();

        ros::spinOnce();
        rate.sleep();
    }

    //降落
    bp1_land_pub.publish(std_msgs::Empty());
    ros::Duration(3).sleep();
    bp1_land_pub.publish(std_msgs::Empty());

    ros::spinOnce();
    printf_state();
    rate.sleep();

    return 0;
}

double current_time()
{ros::Duration elapsed_time = ros::Time::now() - start_time;
    double secs =elapsed_time.toSec();
    return secs;
}


Eigen::Vector3d setDronePosition(const geometry_msgs::Point pose_drone)
{ Eigen::Vector3d drone_pos;
    drone_pos(0)=pose_drone.x;
    drone_pos(1)=pose_drone.y;
    drone_pos(2)=pose_drone.z;
    return drone_pos;
}

Eigen::Vector3d QuaterniondToEuler(const geometry_msgs::Quaternion quaternion_drone)
{
    Eigen::Quaterniond q;  //四元数
    q.x()=quaternion_drone.x;
    q.y()=quaternion_drone.y;
    q.z()=quaternion_drone.z;
    q.w()=quaternion_drone.w;
    q=q.coeffs().normalized();
    Eigen::Matrix3d Rx=q.toRotationMatrix();  //旋转矩阵
    Eigen::Vector3d ea=Rx.eulerAngles(2,1,0); //欧拉角   yaw--1??
    if ( (abs(ea[1])>pi/2) | (abs(ea[2])>pi/2) )
    {ea[0]=ea[0]-pi;
    }
    return ea;
}

void printf_state()
{ cout <<endl<<">>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<" <<endl;
    cout <<"X, Y, Z, yaw: "<< bp1_vrpn_odom.pose.position.x << ", "<<bp1_vrpn_odom.pose.position.y<<", "<<bp1_vrpn_odom.pose.position.z<<", "<< bp1_current_euler[0]*180/pi << endl;
    cout <<"t, x, y, z: " <<t<<", " << current_x << ", "<< current_y <<", "<< z << endl;
}
