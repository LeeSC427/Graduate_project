#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"
#include <string>
#include <signal.h>
#include <vector>

class Point_cloud
{
    public:
        int point_x = 0;
        int point_y = 0;
        double ang = 0.0;
        int center_x = 240;
        int center_y = 320;
        
        
        void point_cloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
        {
            cv::Point center(center_x, center_y);
            cv::Mat mat(640, 480, CV_8UC3, cv::Scalar(128, 128, 128));
            for(int i = 0; i < scan_msg->ranges.size(); i++)
            {

                ang = scan_msg->angle_increment * i;
                point_x = 240 + std::round(sin(ang)*scan_msg->ranges[i] * 30);
                point_y = 320 + std::round(cos(ang)*scan_msg->ranges[i] * 30);

                cv::Point pt(point_x, point_y);
                cv::circle(mat, pt, 4, cv::Scalar(0,0,0), -1);
                cv::line (mat, center, pt, cv::Scalar(255,255,255),4);
                // std::cout << point_x << " ";
                // point_y = std::round(cos(ang)*scan_msg->ranges[i]*30) + 320;
                // point_x = std::round(sin(ang)*scan_msg->ranges[i]*30) + 240;
                
                // cv::Point pt_cir(point_x, point_y);
                // cv::circle(mat, pt_cir, 3, cv::Scalar(0, 0, 0), -1);
                // cv::line(mat,center, pt_cir, cv::Scalar(255,255,255), 3);
            }
                cv::namedWindow("Point Cloud");
                cv::imshow("Point Cloud", mat);
                cv::waitKey(1);
        }
};