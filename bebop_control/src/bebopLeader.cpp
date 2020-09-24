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
    cout <<"bebop  init! " << endl;

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

    pidX.outMax = 3.6;
    pidX.outMin = -3.6;
    pidY.outMax = 3.6;
    pidY.outMin = -3.6;

    pidZ.kp = 0.6;
    pidZ.kd = 0.2;

    pidTheta.kp = 0.05;

    pi = 3.1415926;
    //bebop
    takeoff_pub= nh.advertise<std_msgs::Empty>("bebop4/takeoff", 1);         // 发布 起飞命令
    land_pub   = nh.advertise<std_msgs::Empty>("bebop4/land", 1);            // 发布 降落命令
    cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop4/cmd_vel", 1);    // 发布 移动命令
    vrpn_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/bebop4/pose", 1, &bebopControl::vrpn_cb,this);
    leaderPosition_pub = nh.advertise<bebop_control::leaderPosition>("leader/position",1);


    max_tilt_angle=0.1;   //  line.x, line.y
    max_vert_speed=1;   //  line.z
    max_rot_speed=10;    //  angular.z

}
/*起飞指令*/
void bebopControl::takeoff()
{
    takeoff_pub.publish(std_msgs::Empty());
    cout <<"bebop  takeoff! " << endl;
}
/*降落指令*/
void bebopControl::land()
{
    land_pub.publish(std_msgs::Empty());
    cout <<"bebop  land! " << endl;
}
/*发布位置控制指令*/
void bebopControl::cmd(float x,float y,float z)
{
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.linear.z = z;
    cmd_pub.publish(cmd_vel);
    cout <<"bebop  send cmd! " << endl;
}
/*通过vrpn接受bebop位置消息*/
void bebopControl::vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*Motive通过 VRPN 发布的位置消息 单位是 米
      ×1000 转化为mm
    */
    // currentPos.x = msg->pose.position.x;
    // currentPos.y = msg->pose.position.y;
    // currentPos.z = msg->pose.position.z;

    vrpn_odom = *msg;
    cout <<"vrpn position:  " << vrpn_odom.pose.position.x  <<"   " << vrpn_odom.pose.position.y <<"   " << vrpn_odom.pose.position.z << endl;
}
void bebopControl::start()
{
    double t;
    double setupTime = 8;
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
    targetPos.z = 1.2;

    ros::spinOnce();        
    rate->sleep();

    currentPos = getPosition(vrpn_odom.pose.position);   

    start_time = ros::Time::now();

    while(ros::ok())
    {

        t =  this->current_time();
        

        if(t <= (setupTime) )
        {
            targetPos.x = 0;
            target_v.x = 0;
            target_a.x = 0;

            targetPos.y = -1;
            target_v.y = 0;
            target_a.y = 0;      
        }
        /**************************************************/
        else
        {

            targetPos.x = 1*sin((t-setupTime)/18*pi);
            target_v.x =  1*cos((t-setupTime)/18*pi)*pi/18;
            target_a.x = 0;

            targetPos.y = -1*cos((t-setupTime)/18*pi);
            target_v.y = 1*sin((t-setupTime)/18*pi)*pi/18;
            target_a.y = 0;               
        }    
        // cout << "targetPos is: " << targetPos.x << "  " << targetPos.y  << endl;
        /**************************************************/



        // else if (t< setupTime + 12 * 1) // y往前飞
        // {
        //     targetPos.x = 1.2 - 0.7;
        //     target_v.x = 0;
        //     target_a.x = 0;

        //     targetPos.y = -1.2 + 0.2 * (t - setupTime - 12 * 0);
        //     target_v.y = 0.2;
        //     target_a.y = 0;
        // }
        // else if (t< setupTime + 12 * 1 + 6) // 静止
        // {
        //     targetPos.x = 1.2 - 0.7;
        //     target_v.x = 0;
        //     target_a.x = 0;

        //     targetPos.y = 1.2;
        //     target_v.y = 0;
        //     target_a.y = 0;
        // }           
        // else if (t< setupTime + 12 * 2) // x往后飞
        // {
        //     targetPos.x = 1.2 - 0.2 * (t - setupTime - 12 * 1) - 0.7;
        //     target_v.x = -0.2;
        //     target_a.x = 0;

        //     targetPos.y = 1.2;
        //     target_v.y = 0;
        //     target_a.y = 0;
        // }
        // else if (t< setupTime + 12 * 3) // y往后飞
        // {
        //     targetPos.x = -1.2 - 0.3;
        //     target_v.x = 0;
        //     target_a.x = 0;

        //     targetPos.y = 1.2 - 0.2*(t - setupTime - 12 * 2);
        //     target_v.y = -0.2;
        //     target_a.y = 0;
        // }
        // else if (t< setupTime + 12 * 3 + 6) // 静止
        // {
        //     targetPos.x = -1.2 - 0.3;
        //     target_v.x = 0;
        //     target_a.x = 0;

        //     targetPos.y = -1.2;
        //     target_v.y = 0;
        //     target_a.y = 0;
        // }    
        lastPos = currentPos;
        last_v = current_v;
        /*欧拉角*/
        current_euler = QuaterniondToEuler(vrpn_odom.pose.orientation);

        currentPos = getPosition(vrpn_odom.pose.position); 
        /*向tracker发送位置*/
        leader_position.x = currentPos.x;
        leader_position.y = currentPos.y;
        leader_position.z = currentPos.z;


        current_v.x = (currentPos.x - lastPos.x)*delatT;
        current_v.y = (currentPos.y - lastPos.y)*delatT;
        current_v.z = (currentPos.z - lastPos.z)*delatT;

        current_a.x = (current_v.x - last_v.x)*delatT;
        current_a.y = (current_v.y - last_v.y)*delatT;
        current_a.z = (current_v.z - last_v.z)*delatT;

        leader_position.vx = current_v.x;
        leader_position.vy = current_v.y;
        leader_position.vz = current_v.z;
        leaderPosition_pub.publish(leader_position);

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
            if(t>= 36+setupTime)
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
                cout << "xy is: " << xy[0] << "  " << xy[1] << endl;
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

                // if(xy[0]>3.6)
                //     xy[0]=3.6;
                // else if(xy[0]<-3.6)
                //     xy[0]=-3.6;

                // if(xy[1]>3.6)
                //     xy[1]=3.6;
                // else if(xy[1]<-3.6)
                //     xy[1]=-3.6;

                // if(xy[0]>1)
                //     xy[0]=1;
                // else if(xy[0]<-1)
                //     xy[0]=-1;

                // if(xy[1]>1)
                //     xy[1]=1;
                // else if(xy[1]<-1)
                //     xy[1]=-1;

                // cmd_vel.linear.x =  xy[0];
                // cmd_vel.linear.y =  xy[1];

                cmd_vel.linear.z =  pidZ.out * max_vert_speed;
                cmd_vel.angular.z= dyaw * max_rot_speed;         

                cmd_pub.publish(cmd_vel);
            }     
        }

        // cout <<"current_euler:  " << current_euler[0]  <<"   " << current_euler[1] <<"   " << current_euler[2] << endl;
        /*高度小于55cm直接降落 
          注意：  Motive的y轴是垂直向上的
        */
        // if(currentPos.y<=550 && currentPos.y>0)
        // {
        //     this->land();
        //     break;
        // }
        // /*以0.1m/s的速度下降*/
        // this->cmd(0,0,-0.1);

		// if(ros::param::get("k_r", kr))
		// 	cout << "parameter kr = " << kr << endl;        
        // else
        //  {
        //      kr = 1.5;
        //      cout << "parameter error!!" << endl; 
        //  } 
		// if(ros::param::get("k_v", kv))
        //     cout << "parameter kv = " << kv << endl;         
        // else
        //  {
        //      kv = 2.5;
        //      cout << "parameter error!!" << endl; 
        //  }          

        ros::spinOnce();
        rate->sleep();
    }
        this->land();
        ros::Duration(3).sleep();
        this->land();
}



int main(int argc, char **argv)
{   
    ros::init(argc, argv, "bebopLeader");
    
    bebopControl bebopLeader;
    bebopLeader.start();

    return 0;
}

