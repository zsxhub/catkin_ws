#include <ros/ros.h>  
#include <px4_offb/setPosition.h>  
#include <px4_offb/setActivity.h> 


class pos_control
{
public:
    pos_control();

    ros::NodeHandle nh;//p创建句柄
    ros::Publisher positionPub;
    ros::Publisher activityPub;

    px4_offb::setPosition position;
    px4_offb::setActivity activity;

    void pubPosition(float x,float y,float z);
    void pubActivity(void);
    void start(void);

private:
    /* data */
};

pos_control::pos_control()
{
    activity.activity = "land";

    
    positionPub = nh.advertise<px4_offb::setPosition>("position_info",1);
    activityPub = nh.advertise<px4_offb::setActivity>("Activity_info",1);

}
void pos_control::pubPosition(float x,float y,float z)
{
    position.x = x;
    position.y = y;
    position.z = z;
    positionPub.publish(position);
}

void pos_control::pubActivity(void)
{
    activityPub.publish(activity);
}

void pos_control::start(void)
{
    ros::Duration(2).sleep();
    this->pubPosition(0,0,2);
    ros::Duration(5).sleep();
    this->pubPosition(0,0,3);
    ros::Duration(5).sleep();
    this->pubPosition(3,0,3);
    ros::Duration(5).sleep();
    this->pubPosition(3,3,3);
    ros::Duration(5).sleep();
    this->pubPosition(0,3,3);
    ros::Duration(5).sleep();
    this->pubPosition(0,0,3);
    ros::Duration(5).sleep();
    this->pubActivity();
    ros::Duration(2).sleep();
}
int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "px4_pos_control");

    pos_control posContral;
    posContral.start();

    return 0;
}