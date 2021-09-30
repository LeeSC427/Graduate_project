#pragma once

#include <cstring>
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
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <mutex>



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

class Points
{
    public:
        int point_x;
        int point_y;
        cv::Point poi;
        void points(int x, int y)
        {
            this->point_x = x;
            this->point_y = y;
            this->poi = cv::Point(x,y);
        }
};

class Pnt_raw
{
    public:
        float raw_x;
        float raw_y;
        cv::Point2f pnt_raw;
        void point_raw(float x, float y)
        {
            this->raw_x = x;
            this->raw_y = y;
            this->pnt_raw = cv::Point2f(x,y);
            // std::cout << "raw_x: " << raw_x << std::endl;
        }

};



class Targets
{
    public:
        int target_x;
        int target_y;
        cv::Point targ;
        void targets(int x, int y)
        {
            this->target_x = x;
            this->target_y = y;
            this->targ = cv::Point(x,y);
        }
};

class Mean
{
    public:
        int mean_x;
        int mean_y;
        cv::Point MEAN;
        void mean(int x, int y)
        {
            this->mean_x = x;
            this->mean_y = y;
            this->MEAN = cv::Point(x,y);
        }
};

class Mean_raw
{
    public:
        float mean_raw_x;
        float mean_raw_y;
        cv::Point2f MEAN_RAW;

        void mean_raw(float x, float y)
        {
            this->mean_raw_x = x;
            this->mean_raw_y = y;
            this->MEAN_RAW = cv::Point2f(x,y);
        }
};

class Point_cloud
{
    public:
            std::mutex mtx;
            int point_x = 0;
            int point_y = 0;
            float point_x_raw = 0.0;
            float point_y_raw = 0.0;
            int center_x = 240;
            int center_y = 320;
            int box_dist = 20;
            double ang = 0.0;
            int tar_radius = 8;
            int tar_center = 3;
            double p2p_dist = 0.0;
            int runtime = 0;
            int& ref_run = runtime;
            Mean mean;
            Mean_raw mean_raw;
            vector<Targets> TARGET1;
            vector<Pnt_raw> TARGET_RAW1;
            std::string win_name;
            float m_to_pixel = 50.0;
            float pixel_to_m = 1.0/50.0;
            Tf_val tf_val;
            ros::Publisher tar_pos;
            float tar_pub_x = 0.0;
            float tar_pub_y = 0.0;
            
        void get_name(std::string name)
        {
            win_name = name;
        }

        void publish(ros::NodeHandle n)
        {
            tar_pos = n.advertise<nav_msgs::Odometry>("tar_pos", 100);
        }

        void pub_tar_pos(Mean_raw mean)
        {
            nav_msgs::Odometry tar;

            tar.header.stamp = ros::Time::now();
            tar.header.frame_id = "/laser_multi";

            tar.pose.pose.position.x = mean.mean_raw_x ;
            tar.pose.pose.position.y = mean.mean_raw_y ;
            tar.pose.pose.position.z = 0.0;
            
            // std::cout << "********************************************" << std::endl;
            // std::cout << "pub_tar_pos_mean (x,y): " << mean.mean_raw_x << "," << mean.mean_raw_y << std::endl;
            // std::cout << "********************************************" << std::endl;

            // std::cout << "pub_tar_pos" << std::endl;


            tar_pos.publish(tar);
        }

        void Tracking(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
        {
            mtx.lock();

            vector<Points> POINTS1;
            vector<Points> POINTS;
            vector<Targets> TARGET2;
            vector<Targets> TARGET;
            vector<Pnt_raw> POINTS_RAW;
            vector<Pnt_raw> POINTS_RAW1;
            vector<Pnt_raw> TARGET_RAW;

            int size = scan_msg->ranges.size();
            double scan_angle = scan_msg->angle_increment;
            Targets tar;
            Pnt_raw raw;

            cv::Point center(center_x, center_y);
            cv::Mat mat(640, 480, CV_8UC3, cv::Scalar(128, 128, 128));
            std::vector<float> scan_range(size);
            std::vector<float> scan_range_raw(size);
            
            POINTS.clear();
            scan_range.clear();
            POINTS_RAW.clear();
            
            // std::cout << "[";
           
            for(int i = 0; i < size; i++)
            {
                scan_range.push_back(scan_msg->ranges[i]);
                // scan_range_raw.push_back(scan_msg->ranges[i]);
            }
            scan_range_raw = scan_range;

            // for(int i = 0; i < size; i++)
            // {
            //     std::cout << scan_range_raw[i] << "," ;
            // }

            // std::cout << "]" << std::endl;

            
            POINTS1 = point_cloud( POINTS, center, mat, scan_range, scan_angle,size);
            POINTS_RAW1 = point_cloud_raw(POINTS_RAW, scan_range_raw, scan_angle, size);

            Center(center, mat);
            
            Box(mat);
            std::cout << TARGET1.size() << std::endl;

            if(TARGET1.size() == 0)
                runtime = 0;


            if(runtime == 0)
            {
                TARGET1 = Target_initialize(mat, tar, TARGET, POINTS1, size, ref_run);
                TARGET_RAW1 = Target_init_raw(raw, TARGET_RAW, POINTS1, POINTS_RAW1, size);
            }

            else if(runtime == 1)
            {
                TARGET1 = New_target(mat, mean, tar, TARGET1, POINTS1, size, mat, ref_run);
                TARGET_RAW1 = New_target_raw(mean, raw, TARGET_RAW1, POINTS1, POINTS_RAW1, size);
            }
            // std::cout << mean.MEAN << std::endl;
           
            int tar_size = TARGET1.size();
            int tar_raw_size = TARGET_RAW1.size();

            // std::cout << "size = " << TARGET_RAW1.size() << std::endl;

            if(tar_size > 0)
            {
                mean = MEAN(mat, TARGET1, mean, tar_size);
                mean_raw = MEAN_raw(TARGET_RAW1, mean_raw, tar_raw_size);
            }



            Show(mat, win_name);

            mtx.unlock();
        }

        std::vector<Points> point_cloud( vector<Points> POINTS, cv::Point center, cv::Mat matrix, std::vector<float> scan_range, double scan_angle, int size)
        {
            Points po;
            Points raw;
            for(int i = 0; i < size; i++)
            {
                if(scan_range[i] <= 50.0)
                {
                    ang = scan_angle * i;
                    
                    point_x = sin(ang)*scan_range[i] * m_to_pixel;
                    point_y = cos(ang)*scan_range[i] * m_to_pixel;

                    
                }
                else
                {
                    point_x = 0;
                    point_y = 0;
                }
                
                tf::Vector3 Original_point = tf::Vector3(point_x, point_y, 0);  
                tf::Vector3 transform_point = Original_point * tf_val.R + tf_val.T;

                
                po.points(std::round(transform_point[0]), std::round(transform_point[1]));
                POINTS.push_back(po);

                cv::line(matrix, center, POINTS[i].poi, cv::Scalar(255,255,255),3);
                cv::circle(matrix, POINTS[i].poi, 2, cv::Scalar(0,0,0), -1);
            }
            
            return POINTS;
        }

        std::vector<Pnt_raw> point_cloud_raw(vector<Pnt_raw> POINTS_RAW, std::vector<float> scan_range, double scan_angle, int size)
        {
            Pnt_raw raw;
        
            for(int i = 0; i < size; i++)
            {
                if(scan_range[i] <= 50.0)
                {
                    ang = scan_angle * i;
                    
                    point_x_raw = (-1.0) * cos(ang)*scan_range[i];
                    point_y_raw = (-1.0) * sin(ang)*scan_range[i];

                    
                }
                else
                {
                    point_x_raw = 0;
                    point_y_raw = 0;
                }
                
                tf::Vector3 Original_point = tf::Vector3(point_x_raw, point_y_raw, 0);
                tf::Vector3 transform_raw = Original_point * tf_val.R;
                // std::cout << "-----------" << transform_raw[0] << "," << transform_raw[1] << "--------" << std::endl;
                raw.point_raw(transform_raw[0],transform_raw[1]);
                POINTS_RAW.push_back(raw);
            }
            
            return POINTS_RAW;
        }
    
        void Center(cv::Point center, cv::Mat matrix)
        {
            cv::circle(matrix, center, 2, cv::Scalar(0,0,0), -1);
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

        std::vector<Targets> Target_initialize(cv::Mat mat, Targets tar, vector<Targets> TARGET, vector<Points> POINTS, int size, int &ref_run)
        {
            int a = 0;
            int b = 0;
            int c = 0;
            Mean mean1;
            
            for(int i = 0; i < size; i++)
            {
                if(POINTS[i].point_x > 245 || POINTS[i].point_x < 235)
                {
                    if(POINTS[i].point_y > 325 || POINTS[i].point_y < 315)
                    {
                            ++a;
                        if(((center_x - box_dist) < POINTS[i].point_x) && (POINTS[i].point_x < (center_x + box_dist)))
                        {
                                ++b;
                            if((center_y - box_dist) < POINTS[i].point_y && POINTS[i].point_y < (center_y + box_dist))
                            {
                                    ++c;
                                    std::cout << "target initialize" << std::endl;
                                    tar.targets(POINTS[i].point_x, POINTS[i].point_y);
                                    TARGET.push_back(tar);
                                    ref_run = 1;
                            }
                        }
                    }
                }                    
            }            

            return TARGET;

        }

        std::vector<Pnt_raw> Target_init_raw(Pnt_raw raw, vector<Pnt_raw> TARGET_RAW, vector<Points> POINTS, vector<Pnt_raw> POINTS_RAW1, int size)
        {
            int a = 0;
            int b = 0;
            int c = 0;
            Mean mean1;
            
            for(int i = 0; i < size; i++)
            {
                if(POINTS[i].point_x > 245 || POINTS[i].point_x < 235)
                {
                    if(POINTS[i].point_y > 325 || POINTS[i].point_y < 315)
                    {
                        ++a;
                        if(((center_x - box_dist) < POINTS[i].point_x) && (POINTS[i].point_x < (center_x + box_dist)))
                        {
                                ++b;
                            if((center_y - box_dist) < POINTS[i].point_y && POINTS[i].point_y < (center_y + box_dist))
                            {
                                    ++c;
                                    std::cout << "target initialize" << std::endl;
                                    raw.point_raw(POINTS_RAW1[i].raw_x, POINTS_RAW1[i].raw_y);
                                    // std::cout << "---------------------------------------------" << std::endl;
                                    // std::cout << "Target (x, y):" << POINTS_RAW1[i].raw_x << "," << POINTS_RAW1[i].raw_y << std::endl;
                                    TARGET_RAW.push_back(raw);
                            }
                        }
                    }
                }     
            }               
            return TARGET_RAW;

        }

        Mean MEAN(cv::Mat mat, vector<Targets> &TARGET, Mean mean, int size)
        {
            double sum_x = 0;
            double sum_y = 0;
            int div_x;
            int div_y;

            if(size > 0)
            {
                for(int i = 0; i < size; i++)
                {
                    sum_x = sum_x + TARGET[i].target_x;
                    sum_y = sum_y + TARGET[i].target_y;
                }
                div_x = sum_x/size;
                div_y = sum_y/size;
                mean.mean(div_x, div_y);
            }
            else
                std::cout << "no target" << std::endl;
            
            std::cout << TARGET1.size() << std::endl;
            
            
            cv::circle(mat, mean.MEAN, tar_radius, cv::Scalar(0, 0, 255), 2);
            cv::circle(mat, mean.MEAN, tar_center, cv::Scalar(0, 0, 255),-1);
            return mean;
        }

        Mean_raw MEAN_raw(vector<Pnt_raw> &POINTS_RAW, Mean_raw &mean, int size)
        {
            double sum_x = 0.0;
            double sum_y = 0.0;
            double div_x;
            double div_y;

            if(size > 0)
            {
                for(int i = 0; i < size; i++)
                {
                    sum_x = sum_x + POINTS_RAW[i].raw_x;
                    sum_y = sum_y + POINTS_RAW[i].raw_y;
                    // std::cout << "mean_raw:" << POINTS_RAW[i].raw_x << "," << sum_y << std::endl;
                }
                div_x = sum_x/size;
                div_y = sum_y/size;
                mean.mean_raw(div_x, div_y);
            }
            else
                std::cout << "no target" << std::endl;

            // std::cout << "********************************************" << std::endl;
            // std::cout << "mean (x,y): " << mean.mean_raw_x << "," << mean.mean_raw_y << std::endl;
            // std::cout << "********************************************" << std::endl;
            // std::cout << "mean_div_x: " << div_x << std::endl;
            pub_tar_pos(mean);

            return mean;
        }


        double Distance(Mean mean, int point_x, int point_y)
        {
            double p2p_distance = std::sqrt(std::pow(point_x - mean.mean_x, 2) + std::pow(point_y - mean.mean_y, 2));
            return p2p_distance;
        }

        std::vector<Targets> New_target(cv::Mat mat, Mean mean,Targets tar, vector<Targets> TARGET, vector<Points> POINTS, int size, cv::Mat matrix, int &ref_run)
        {           
            std::vector<Targets> TARGETa;
            TARGETa.clear();
            

            std::cout << "New Target" << std::endl;

            for(int i = 0; i < size; i++)
            {

                p2p_dist = Distance(mean, POINTS[i].point_x, POINTS[i].point_y);
                if(p2p_dist <= tar_radius)
                {
                tar.targets(POINTS[i].point_x, POINTS[i].point_y);
                TARGETa.push_back(tar);        
                }
            }    

            if(TARGETa.size() == 0)
            {
                TARGETa = TARGET;
            }   

            int tar_size = TARGETa.size();

            ref_run = 1;

            TARGET.clear();
            return TARGETa;
        }

        std::vector<Pnt_raw> New_target_raw(Mean mean, Pnt_raw raw, vector<Pnt_raw> TARGET_RAW, vector<Points> POINTS, vector<Pnt_raw> POINTS_RAW1, int size)
        {           
            std::vector<Pnt_raw> POINTS_RAWa(size);
            POINTS_RAWa.clear();
        
            for(int i = 0; i < size; i++)
            {

                p2p_dist = Distance(mean, POINTS[i].point_x, POINTS[i].point_y);
                if(p2p_dist <= tar_radius)
                {
                raw.point_raw(POINTS_RAW1[i].raw_x, POINTS_RAW1[i].raw_y);
                POINTS_RAWa.push_back(raw);        
                }

                // std::cout << "target_x_points:" << POINTS_RAWa[i].raw_x << std:: endl;
            }    

            if(POINTS_RAWa.size() == 0)
            {
                POINTS_RAWa = TARGET_RAW;
            }   

            // int tar_size = POINTS_RAWa.size();

            TARGET_RAW.clear();
            return POINTS_RAWa;
        }
        
        void Show(cv::Mat matrix, std::string name_win)
        {
            cv::namedWindow(name_win);
            cv::imshow(name_win, matrix);
            cv::waitKey(1);
        }

};