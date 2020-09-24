#include <ros/ros.h>  
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include<image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库

#include<iostream> //C++标准输入输出库
using namespace std;

//图片存放路径
string filepath = "/home/linux/work/catkin_ws/src/opcv_ros/src/1.png";

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publishImage1");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //设置发布主题 camera/image
    image_transport::Publisher pub = it.advertise("/bebop5/image_raw",1);

    cv::Mat capimage;
    cv::VideoCapture cap;
    cap.open(1);
    cv::startWindowThread();

    ros::Rate loop_rate(10);
    while (nh.ok())
    {
        if(cap.isOpened())
        {
            cap >> capimage;
            cv::imshow("capimage",capimage);
            cv::waitKey(5);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",capimage).toImageMsg();
            pub.publish(msg);

        }    
        ros::spinOnce();
        loop_rate.sleep();        
    }
    return 0;
}

