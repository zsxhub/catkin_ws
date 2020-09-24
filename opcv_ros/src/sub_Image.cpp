#include <ros/ros.h>  
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include<image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库

#include <opencv2/features2d/features2d.hpp>
#include<iostream> //C++标准输入输出库


using namespace std;


cv::Mat sub1RecImage ;
cv::Mat sub2RecImage ;
bool sub1RecFlag = false;
bool sub2RecFlag = false;
//消息订阅回调函数
void sub1Image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sub1RecImage = cv_bridge::toCvShare(msg,"bgr8")->image;
        sub1RecFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "sub1Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}
void sub2Image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sub2RecImage = cv_bridge::toCvShare(msg,"bgr8")->image;
        sub2RecFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "sub2Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}

int imagePress(cv::Mat img1,cv::Mat img2)
{
    cv::Mat outimg;
    bool try_use_gpu = false;
    

    // std::vector<cv::KeyPoint> keypoints1;
    // std::vector<cv::KeyPoint> keypoints2;
    // cv::Mat descriptors1;
    // cv::Mat descriptors2;

    // cv::Mat images;
    // if (img1.empty() || img2.empty())
    // {
    //     cout << "Can't read image" << endl;
    //     return -1;
    // }
    // images.push_back(img1);
    // images.push_back(img2);

    // Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
    
    // cv::cvtColor(img,outimg,cv::COLOR_BGR2GRAY);

    cv::imshow("ORB features",outimg);
    cv::waitKey(5);
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscribImage");
    
    ros::NodeHandle nh;//创建句柄
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub1;
    image_transport::Subscriber sub2;

    //设置订阅主题 camera/image
    sub1 = it.subscribe("camera1/image",1,sub1Image_cb);
    sub2 = it.subscribe("camera2/image",1,sub2Image_cb);

    ros::Rate loop_rate(50);

    // 读取图片进行测试
    std::string img1path =  "/home/linux/work/catkin_ws/src/opcv_ros/src/9.png";
    std::string img2path =  "/home/linux/work/catkin_ws/src/opcv_ros/src/10.png";
    cv::Mat img1;
    cv::Mat img2;

    // 读取电脑摄像头
    cv::VideoCapture cap;
    cv::startWindowThread();
    cap.open(1);
    
    while (ros::ok())
    {

        if(cap.isOpened())
        {
            cap >> img1;
            img2 = img1 ;
        }    

        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
