#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"
#include <string>
#include <signal.h>
#include <vector>

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

class Point_cloud
{
    public:
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
            vector<Targets> TARGET1;

        void Tracking(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
        {
            vector<Points> POINTS1;
            vector<Points> POINTS;
            vector<Targets> TARGET2;
            vector<Targets> TARGET;
            int size = scan_msg->ranges.size();
            double scan_angle = scan_msg->angle_increment;
            Targets tar;

            cv::Point center(center_x, center_y);
            cv::Mat mat(640, 480, CV_8UC3, cv::Scalar(128, 128, 128));
            std::vector<float> scan_range(size);
            
            POINTS.clear();
            scan_range.clear();
           
            for(int i = 0; i < size; i++)
            {
                scan_range.push_back(scan_msg->ranges[i]);
            }
            
            POINTS1 = point_cloud(POINTS, center, mat, scan_range, scan_angle,size);


            Center(center, mat);
            
            Box(mat);
            std::cout << TARGET1.size() << std::endl;

            if(TARGET1.size() == 0)
                runtime = 0;


            if(runtime == 0)
            {
                TARGET1 = Target_initialize(mat, tar, TARGET, POINTS1, size, ref_run);
            }

            else if(runtime == 1)
            {
                TARGET1 = New_target(mat, mean, tar, TARGET1, POINTS1, size, mat, ref_run);
            }
            std::cout << mean.MEAN << std::endl;
           
            int tar_size = TARGET1.size();
            
            if(tar_size > 0)
            {
                mean = MEAN(mat, TARGET1, mean, tar_size);
            }

            Show(mat);
        }

        std::vector<Points> point_cloud(vector<Points> POINTS, cv::Point center, cv::Mat matrix, std::vector<float> scan_range, double scan_angle, int size)
        {
            Points po;
            for(int i = 0; i < size; i++)
            {
                ang = scan_angle * i;
                po.points(240 + std::round(sin(ang)*scan_range[i] * 50), 320 + std::round(cos(ang)*scan_range[i] * 50));
                POINTS.push_back(po);

                cv::line(matrix, center, POINTS[i].poi, cv::Scalar(255,255,255),3);
                cv::circle(matrix, POINTS[i].poi, 2, cv::Scalar(0,0,0), -1);
            }
            
            return POINTS;
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
                if(POINTS[i].point_x != 240 && POINTS[i].point_y != 320)
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

            return TARGET;

        }

        Mean MEAN(cv::Mat mat, vector<Targets> TARGET, Mean mean, int size)
        {
            int sum_x = 0;
            int sum_y = 0;
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
        
        void Show(cv::Mat matrix)
        {
            cv::namedWindow("Point Cloud");
            cv::imshow("Point Cloud", matrix);
            cv::waitKey(1);
        }

};