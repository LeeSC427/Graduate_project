#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

class Coordinate
{
    public:
        int first;
        int second;
    
    void Point(cv::Mat matrix)
    {   
        cv::Point pt(first, second);
        cv::circle(matrix, pt, 3, cv::Scalar(0, 0, 0), -1);
    }

    void Show(cv::Mat matrix)
    {
        cv::imshow("Coordinate", matrix);
    }

    Coordinate()
    {
        std::cout << "1st_co: ";
        std::cin >> first;
        std::cout << "2nd_co: ";
        std::cin >> second;

    }
    ~Coordinate()
    {

    }
};

int main()
{
    Coordinate co;

    while(1)
    {
    cv::Mat mat(640, 480, CV_8UC3, cv::Scalar(255,255,255));
    co.Point(mat);
    co.Show(mat);
    
    if(cv::waitKey(1)==27)
    {
        break;
    }
    
    }
    return 0;
}