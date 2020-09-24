#include <ros/ros.h>
#include <topic_demo/gps.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//与按键读取合用
// #include <key_read/keyValue.h> 
// #include </home/linux/work/catkin_ws/src/vrpn_client_ros/vrpn_client_ros.h>

// void gpsCallback(const topic_demo::gps::ConstPtr &msg)
// {
//     std_msgs::Float32 distance;
//     distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
//     ROS_INFO("Listener:Distance to origin = %f,state:%s",distance.data,msg->state.c_str());

// }

// int main(int argc, char *argv[])
// {
//     /* code for main function */
//     ros::init(argc, argv, "listener");
//     ros::NodeHandle listenerHandle;
//     ros::Subscriber sub = listenerHandle.subscribe("gps_info", 1, gpsCallback);
//     ros::spin();
//     return 0;
// }

void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    ROS_INFO("vrpn position:%f  %f  %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "listener");
    ros::NodeHandle listenerHandle;
    ros::Subscriber sub = listenerHandle.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/bebop5/pose", 1, gpsCallback);
    ros::spin();
    return 0;
}
