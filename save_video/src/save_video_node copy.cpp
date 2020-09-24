//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>     
#include<iostream>

#include <ros/ros.h> 
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    //视频保存位置
    string saveVideoPath = "/home/linux/data/dataset/saveVideo/test.avi";  

    //打开摄像头
    // cv::VideoCapture cap1;
    // cv::startWindowThread();
    // cap1.open(1); 

    VideoWriter outputVideo;
    //获取当前摄像头的视频信息
    cv::Size S = cv::Size((int)cap1.get(CV_CAP_PROP_FRAME_WIDTH),
                          (int)cap1.get(CV_CAP_PROP_FRAME_HEIGHT));                     
    outputVideo.open(saveVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 30.0, S, true);

    if (!outputVideo.isOpened()) {
        cout << "fail to open!" << endl;
        return -1;
    }

    cv::Mat frameImage;
    int count = 0;

    while(true) {
        //读取当前帧
        cap1 >> frameImage;

        if (frameImage.empty()) break;

        ++count;
        //输出当前帧
        cv::imshow("output", frameImage);
        //保存当前帧
        outputVideo << frameImage;

        if ((char)cvWaitKey(1) == 'q') break;
    }

    std::cout << "TotalFrame: " << count << std::endl;
    return 0;
}
