using namespace std;
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
//topic
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <bebop_control/bebop_control.h>
#include <bebop_control/leaderPosition.h>
bebopControl::bebopControl()
{
    ROS_INFO("bebop  init! ");

    delatT = 20;
    rate = new ros::Rate(delatT);
    currentPos.x = 0;
    currentPos.y = 0;
    currentPos.z = 0;

    lastPos.x = 0;
    lastPos.y = 0;
    lastPos.z = 0;

    targetPos.x = 0;
    targetPos.y = 0;
    targetPos.z = 0;  

    target_v.x = 0;
    target_v.y = 0;
    target_v.z = 0;

    target_a.x = 0;
    target_a.y = 0;
    target_a.z = 0;

    pidX.outMax = 3.6;
    pidX.outMin = -3.6;
    pidY.outMax = 3.6;
    pidY.outMin = -3.6;

    pidZ.kp = 0.6;
    pidZ.kd = 0.2;

    pidTheta.kp = 0.05;
    

    pi = 3.1415926;
    
    max_tilt_angle=0.1;   //  line.x, line.y
    max_vert_speed=1;   //  line.z
    max_rot_speed=10;    //  angular.z

 
    //bebop
    takeoff_pub= nh.advertise<std_msgs::Empty>("bebop2/takeoff", 1);         // 发布 起飞命令
    land_pub   = nh.advertise<std_msgs::Empty>("bebop2/land", 1);            // 发布 降落命令
    cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop2/cmd_vel", 1);    // 发布 移动命令
    vrpn_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/bebop2/pose", 1, &bebopControl::vrpn_cb,this);
    rec_leader_sub = nh.subscribe<bebop_control::leaderPosition>("leader/position", 1, &bebopControl::rec_leader_cb,this);
    
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
void bebopControl::vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("vrpn position:%f  %f  %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    // currentPos.x = msg->pose.position.x;
    // currentPos.y = msg->pose.position.y;
    // currentPos.z = msg->pose.position.z;
    // cout <<"vrpn position:  " << currentPos.x  <<"   " << currentPos.y <<"   " << currentPos.z << endl;
    vrpn_odom = *msg;
    cout <<"vrpn position:  " << vrpn_odom.pose.position.x  <<"   " << vrpn_odom.pose.position.y <<"   " << vrpn_odom.pose.position.z << endl;

}

void bebopControl::rec_leader_cb(const bebop_control::leaderPosition::ConstPtr &msg)
{
    leader_odom = *msg;
    // cout <<"leader position:  " << leader_odom.x  <<"   " << leader_odom.y <<"   " << leader_odom.z << endl;
   
}


void bebopControl::start()
{
    double t;
    double setupTime = 10;

    leader_odom.x = 0;
    leader_odom.y =0;

    ros::Duration(3).sleep();//必须延时一段时间，不然飞不起来
    this->takeoff();        //起飞
    ros::Duration(5).sleep();
    ros::spinOnce();        
    rate->sleep();

    current_v.x = 0;
    current_v.y = 0;
    current_v.z = 0;



    float kr = 1.5;
    float kv = 2.5;
    targetPos.z = 1.0;

    ros::spinOnce();        
    rate->sleep();

    currentPos = getPosition(vrpn_odom.pose.position); 

    start_time = ros::Time::now();

    while(ros::ok())
    {

        t = this->current_time();

        // if (t <setupTime)
        // {
        //     /* code for True */
        // }
        
        // targetPos = getPosition(vrpn_leader_odom.pose.position);
        targetPos.x = leader_odom.x-1;
        targetPos.y = leader_odom.y+1;
        targetPos.z = 0.8;

        target_v.x = leader_odom.vx;
        target_v.y = leader_odom.vy;
        target_v.z = leader_odom.vz;

        lastPos = currentPos;
        last_v = current_v;
        /*欧拉角*/
        current_euler = QuaterniondToEuler(vrpn_odom.pose.orientation);

        currentPos = getPosition(vrpn_odom.pose.position); 

        current_v.x = (currentPos.x - lastPos.x)*delatT;
        current_v.y = (currentPos.y - lastPos.y)*delatT;
        current_v.z = (currentPos.z - lastPos.z)*delatT;

        current_a.x = (current_v.x - last_v.x)*delatT;
        current_a.y = (current_v.y - last_v.y)*delatT;
        current_a.z = (current_v.z - last_v.z)*delatT;

        /*限制飞行范围*/
        if ((currentPos.x>2.2)|(currentPos.x<-3)|
            (currentPos.y>3.2)|(currentPos.y<-3)|
            (currentPos.z>2.5))
        {

                cmd_vel.linear.x =  0;
                cmd_vel.linear.y =  0;
                cmd_vel.linear.z =  0;
                cmd_vel.angular.z=  0;         
                cmd_pub.publish(cmd_vel);           
            this->land();
            cout << "out of range!!!" << endl;
            break;
        }
        else
        {
            if(t>= 45)
            {
                this->land();
                cout << "time out! landing..." << endl;
                break;
            }
            else
            {
                out.x = current_a.x - kr*(currentPos.x - targetPos.x) - kv*(current_v.x - target_v.x);
                out.y = current_a.y - kr*(currentPos.y - targetPos.y) - kv*(current_v.y - target_v.y);
                
                xy << out.x , out.y;
                Rotation_R2B << cos(current_euler[0]),sin(current_euler[0]),-sin(current_euler[0]),cos(current_euler[0]);
                
                xy = Rotation_R2B*xy;

                pidZ.out = pidZ.kp*(targetPos.z - currentPos.z) - pidZ.kd*(current_v.z);

                dyaw = 0 - current_euler[0];
                if (dyaw > pi)
                    dyaw = dyaw - 2*pi;
                else
                    if (dyaw <= -pi)
                        dyaw = dyaw + 2*pi;  

                dyaw = pidTheta.kp * dyaw;    

                cmd_vel.linear.x =  xy[0] * max_tilt_angle;
                cmd_vel.linear.y =  xy[1] * max_tilt_angle;
                cmd_vel.linear.z =  pidZ.out * max_vert_speed;
                cmd_vel.angular.z= dyaw * max_rot_speed;         

                if (cmd_vel.linear.x > 0.8)
                {
                    cmd_vel.linear.x =0.8;
                }
                else if (cmd_vel.linear.x < -0.8)
                {
                    cmd_vel.linear.x = -0.8;
                }

                if (cmd_vel.linear.y > 0.8)
                {
                    cmd_vel.linear.y =0.8;
                }
                else if (cmd_vel.linear.y < -0.8)
                {
                    cmd_vel.linear.y = -0.8;
                }            
                
                cmd_pub.publish(cmd_vel);
            }     
        }
        ros::spinOnce();
        rate->sleep();
    }
    this->land();
    ros::Duration(3).sleep();
    this->land();
}
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "bebopTracker");
    
    bebopControl bebopTracker;
    bebopTracker.start();

    return 0;
}

void bebopControl::pix_controller()
{
    pidX.error = targetPos.x - currentPos.x;
    pidX.error = pidX.error / 1000.0;
    
    pidX.out = pidX.kp * pidX.error + pidX.kd*(pidX.error - pidX.error_last);
    pidX.error_last = pidX.error;
}