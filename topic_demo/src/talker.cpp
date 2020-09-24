#include <ros/ros.h>
#include <topic_demo/gps.h>

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "talker");
    ros::NodeHandle talkerHandle;//创建句柄
    topic_demo::gps msg;
    msg.x = 1.0;
    msg.y = 1.0;
    msg.state = "working";
    //创建 publisher   gps_info为名称
    ros::Publisher pub = talkerHandle.advertise<topic_demo::gps>("gps_info",1);

    ros::Rate loop_rate(1.0);

    while (ros::ok())
    {
        /* code for loop body */

        msg.x = 1.03 * msg.x;
        msg.y = 1.04 * msg.y;
        //打印消息
        ROS_INFO("Talker:GPS:x = %f , y = %f",msg.x , msg.y);

        pub.publish(msg);

        loop_rate.sleep();
    }
    
    return 0;
}