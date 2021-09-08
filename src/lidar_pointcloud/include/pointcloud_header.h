#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"
#include <string>
#include <signal.h>
#include <vector>
#include "tf/transform_listener.h"
#include "tf/tf.h"

class Tf_val
{
    public:
        tf::Matrix3x3 R = tf::Matrix3x3(transform.getRotation());
        tf::Vector3 T = tf::Vector3(240, 320, 0);
        tf::StampedTransform transform;
        // void get_tf(tf::StampedTransform transform)
        // {
        //     R = tf::Matrix3x3(transform.getRotation());
        // }
};

class Point_cloud
{
    public:
        int point_x = 0;
        int point_y = 0;
        double ang = 0.0;
        int center_x = 240;
        int center_y = 320;
        std::string win_name;
        int mm_to_pixel = 30;
        Tf_val tf_val;
        int box_dist = 20;

        void get_name(std::string name)
        {
            win_name = name;
        }

        void point_cloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
        {
            cv::Point center(center_x, center_y);
            cv::Mat mat(640, 480, CV_8UC3, cv::Scalar(128, 128, 128));
            
            for(int i = 0; i < scan_msg->ranges.size(); i++)
            {

                if(scan_msg->ranges[i] <= 50.0)
                {
                    ang = scan_msg->angle_increment * i;
                    
                    // point_x = 240 + std::round(sin(ang)*scan_msg->ranges[i] * mm_to_pixel);
                    // point_y = 320 + std::round(cos(ang)*scan_msg->ranges[i] * mm_to_pixel);

                    point_x = std::round(sin(ang)*scan_msg->ranges[i] * mm_to_pixel);
                    point_y = std::round(cos(ang)*scan_msg->ranges[i] * mm_to_pixel);

                    tf::Vector3 Original_point = tf::Vector3(point_x, point_y, 0);
                    // std::cout << point_x << " ";
                    tf::Vector3 transform_point = Original_point * tf_val.R + tf_val.T;
                    
                    point_x = transform_point[0];
                    point_y = transform_point[1];          
                    

                    cv::Point pt(point_x, point_y);
                    
                    cv::circle(mat, pt, 2, cv::Scalar(0,0,0), -1);
                    cv::line (mat, center, pt, cv::Scalar(255,255,255),2);
                }
                else
                {
                    continue;
                }
                // point_y = std::round(cos(ang)*scan_msg->ranges[i]*30) + 320;
                // point_x = std::round(sin(ang)*scan_msg->ranges[i]*30) + 240;
                
                // cv::Point pt_cir(point_x, point_y);
                // cv::circle(mat, pt_cir, 3, cv::Scalar(0, 0, 0), -1);
                // cv::line(mat,center, pt_cir, cv::Scalar(255,255,255), 3);
            }
                // std::cout << win_name << std::endl;
            cv::circle(mat, center, 2, cv::Scalar(255,0,0), -1);
            Box(mat);
            cv::namedWindow(win_name);
            cv::imshow(win_name, mat);
            cv::waitKey(1);
        }

        void Box(cv::Mat matrix)
        {
            cv::Point point1(center_x - box_dist,center_y - box_dist);
            cv::Point point2(center_x - box_dist,center_y + box_dist);
            cv::Point point3(center_x + box_dist,center_y - box_dist);
            cv::Point point4(center_x + box_dist,center_y + box_dist);

            cv::line (matrix, point1, point2, cv::Scalar(0,0,0),1);
            cv::line (matrix, point2, point4, cv::Scalar(0,0,0),1);
            cv::line (matrix, point3, point4, cv::Scalar(0,0,0),1);
            cv::line (matrix, point3, point1, cv::Scalar(0,0,0),1);
        }

};