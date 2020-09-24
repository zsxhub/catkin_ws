#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <bebop_control/leaderPosition.h>

class bebopControl
{
public:
    bebopControl();

    ros::NodeHandle nh;

    geometry_msgs::Twist cmd_vel;
    
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher cmd_pub;

    ros::Subscriber vrpn_sub;

    ros::Rate *rate;
    ros::Time start_time;

    float delatT;

    struct PID
    {
        float kp;
        float ki;
        float kd;

        float out;
        float outMax;
        float outMin;

        float error;
        float error_last;
        float dt;

        
    };
    PID pidX , pidY,pidZ,pidTheta;
    struct xyz
    {
        float x;
        float y;
        float z;
    };
    xyz currentPos,lastPos,targetPos;
    xyz current_v,last_v,target_v;
    xyz current_a,last_a,target_a;
    xyz out;
    

    float pi;//圆周率
    float dyaw;
    Eigen::Vector3d current_euler;
    geometry_msgs::PoseStamped vrpn_odom;
    Eigen::Matrix2d Rotation_R2B;
    Eigen::Vector2d xy;

    float max_tilt_angle;   //  line.x, line.y
    float max_vert_speed;   //  line.z
    float max_rot_speed;    //  angular.z

    void takeoff();
    void land();
    void cmd(float x,float y,float z);
    void vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    ros::Subscriber rec_leader_sub;
    bebop_control::leaderPosition leader_odom;
    void rec_leader_cb(const bebop_control::leaderPosition::ConstPtr &msg);

    ros::Publisher leaderPosition_pub;
    bebop_control::leaderPosition leader_position;

    void start();
    void pix_controller();

    xyz getPosition(const geometry_msgs::Point position)
    {
        xyz pos;
        pos.x = position.x;
        pos.y = position.y;
        pos.z = position.z;

        return pos;
    }

    double current_time()
    {
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        double secs =elapsed_time.toSec();
        return secs;
    }
    Eigen::Vector3d setDronePosition(const geometry_msgs::Point pose_drone)
    { 
        Eigen::Vector3d drone_pos;
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
        {
            ea[0]=ea[0]-pi;
        }
        return ea;
    }
private:
    /* data */
};