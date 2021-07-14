#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#define white = (255,255,255);
#define black = (0,0,0);

class Point
{
    int public pt_col = 249;
    int public pt_row = 249;
    cv::Point pt(pt_col,pt_row);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"moving_point");

    cv::Mat mat(500,500,CV_8UC1,white);

    cv::circle(mat, Point.pt, 1, black, -1);
    std::cout << mat.rows << mat.cols << std::endl;
    
    return 0;
}


// int main(int argc, char **argv)
// {
//     ros::init(argc,argv,"opencv_camera");
    
//     cv::VideoCapture cap(2); //read from the camera #1 CAP_DSHOW: can get FHD image
//     /*
//     make the image to have resolution of 1920*1080
//     */ 
//     cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
//     cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

//     //.isOpened(): check if the camera is connected, and can read the information of it
//     if (!cap.isOpened())    //return 1 if there is an information
//     {
//         std::cout << "Can't open the camera" << std::endl;
//         return -1;  //notify the user there is an error
//     }

//     cv::Mat img;

//     while(1)    //continuously use the camera
//     {
//         cap >> img; //set camera ready, allow to store the information of camera at the array 'img'
//         imshow("camera img", img);
        
//         if(cv::waitKey(1)==27)  //pressing 'esc' will terminate the program
//             break;
//     }
    
//     return 0;
// }
