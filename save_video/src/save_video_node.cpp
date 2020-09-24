//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>     
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>

#include <ros/ros.h> 
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include<image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息
#include<cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库

using namespace std;
using namespace cv;


cv::Mat sub1RecImage ;
cv::Mat sub2RecImage ;
bool sub1RecFlag = false;
bool sub2RecFlag = false;

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



int main(int argc, char *argv[])
{
    //视频保存位置
    string saveVideoPath = "/home/linux/data/dataset/saveVideo/bebop4.avi";  


    ros::init(argc, argv, "save_video_node");
    
    ros::NodeHandle nh;//创建句柄
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub1;
    image_transport::Subscriber sub2;

    //设置订阅主题 camera/image
    sub1 = it.subscribe("/bebop5/image_raw",1,sub1Image_cb);
    sub2 = it.subscribe("camera2/image",1,sub2Image_cb);

    ros::Rate loop_rate(30);

    VideoWriter outputVideo;
    //获取当前摄像头的视频信息
    // cv::Size S = cv::Size((int)cap1.get(CV_CAP_PROP_FRAME_WIDTH),
    //                       (int)cap1.get(CV_CAP_PROP_FRAME_HEIGHT));    
    cv::Size S = cv::Size(856,480);                   
    outputVideo.open(saveVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 20.0, S, true);

    if (!outputVideo.isOpened()) {
        cout << "fail to open!" << endl;
        return -1;
    }

    cv::Mat frameImage;
    int count = 0;

    while(ros::ok()) 
    {
        if (sub1RecFlag == true )
        {
            sub1RecFlag = false;
            sub2RecFlag = false;

            if (sub1RecImage.empty()) break;
            ++count;
            //输出当前帧
            cv::imshow("output", sub1RecImage);
            //保存当前帧
            outputVideo << sub1RecImage;

            if ((char)cvWaitKey(1) == 'q') break;
            cv::waitKey(1);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "TotalFrame: " << count << std::endl;
    return 0;
}
