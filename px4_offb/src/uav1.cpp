/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <px4_offb/setPosition.h> 
#include <px4_offb/setActivity.h> 

class px4_movros
{
public:
    px4_movros();

    /*位置控制信息*/
    geometry_msgs::PoseStamped pose;

    geometry_msgs::PoseStamped localPose;

    bool island;

    mavros_msgs::CommandBool arm_cmd;
    /*px4当前状态*/
    mavros_msgs::State current_state;

    mavros_msgs::SetMode offb_set_mode;

    ros::Rate *rate;

    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber position_sub;
    ros::Subscriber activity_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;


    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setPosition_cb(const px4_offb::setPosition::ConstPtr &msg);
    void setActivity_cb(const px4_offb::setActivity::ConstPtr &msg);
    void waitConnect();
    bool arm();
    bool disarm();
    bool offboard();
    void start();

};
/* 构造函数 */
px4_movros::px4_movros()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    island = false;

    pose.pose.position.x = 6;
    pose.pose.position.y = -3;
    pose.pose.position.z = 2;

    rate = new ros::Rate(10.0);
    /*详细参考 http://wiki.ros.org/mavros#Utility_commands */
    /*FCU state*/
    state_sub = nh.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &px4_movros::state_cb,this);
    /*Local frame setpoint position. NED坐标系(惯性系)*/    
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
    /*Change Arming status. */
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
    /*Set FCU operation mode*/
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
    /*Local position from FCU. NED坐标系(惯性系)*/ 
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10,&px4_movros::local_pose_cb,this); 
    /*订阅位置设置消息*/
    position_sub = nh.subscribe<px4_offb::setPosition>("position_info", 1, &px4_movros::setPosition_cb,this);

    activity_sub = nh.subscribe<px4_offb::setActivity>("Activity_info", 1, &px4_movros::setActivity_cb,this);
}


void px4_movros::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void px4_movros::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    localPose.pose.position.z = msg->pose.position.z;
}
/*处理收到的位置设置消息*/
void px4_movros::setPosition_cb(const px4_offb::setPosition::ConstPtr &msg)
{
    ROS_INFO("set position:%f  %f  %f",msg->x,msg->y,msg->z);
    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = msg->z;
}
void px4_movros::setActivity_cb(const px4_offb::setActivity::ConstPtr &msg)
{
    ROS_INFO("Received Custom Activity:%s",msg->activity.c_str());
    if (strcmp(msg->activity.c_str(),"land") ==0)
    {
        ROS_INFO("px4 is landing");
        island = true;
        pose.pose.position.z = 0.1;
        arm_cmd.request.value = false;
    }
}
void px4_movros::waitConnect()
{
    ROS_INFO("wait for FCU connection...");
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate->sleep();
    }
    ROS_INFO("FCU connection ok!");
}
bool px4_movros::arm()
{
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    } 
    else
        return false;
}
bool px4_movros::disarm()
{
    arm_cmd.request.value = false;
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    } 
    else
        return false;
}
bool px4_movros::offboard()
{
    int i = 100;
    for(i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);

        /* 当前不是 OFFBOARD 模式 */
        if( current_state.mode != "OFFBOARD")
        {
            /*设置 OFFBOARD 模式*/
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
        } 
        else 
        {
            /* 当前未解锁 */
            if( !current_state.armed)
            {
                if(this->arm())
                {
                    ROS_INFO("Vehicle armed");    
                }
            }
        }
        /*解锁成功提前退出*/
        if(current_state.mode == "OFFBOARD" && current_state.armed)
            return true;
        ros::spinOnce();
        rate->sleep();
    }
    return false;
}
void px4_movros::start()
{
    if(!this->offboard())
    {
        while (ros::ok())
        {
            ROS_INFO("Offboard enabled ! Vehicle armed failed ! ");
            ros::spinOnce();
            rate->sleep();
        }
    }
    while(ros::ok())
    {
        local_pos_pub.publish(pose);

        if (island == true && localPose.pose.position.z<=0.15)
        {    
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle diarmed");
                break;
            }
        }
        ros::spinOnce();
        rate->sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav1_offb");

    px4_movros px4Movros;
    px4Movros.start();

    return 0;
}