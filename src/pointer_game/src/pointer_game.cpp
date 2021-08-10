#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <iostream>

class Pnt_game
{
    public:
    
        int pt_cir_col = 249;
        int pt_cir_row = 249;
        int pt_line_row = 499;
        
        void mv_pt(cv::Mat matrix, int key)
        {

            if(key==119) //w
            {
                if(pt_cir_row == 0)
                    pt_cir_row = 499;
                else
                    pt_cir_row--;
            }
            
            else if(key==97) //a
            {
                if(pt_cir_col == 0)
                    pt_cir_col = 499;
                else
                    pt_cir_col--;
            }

            else if(key==115)   //s
            {
                if(pt_cir_row == 499)
                    pt_cir_row = 0;
                else
                    pt_cir_row++;
            }

            else if(key==100) //d
            {
                if(pt_cir_col == 499)
                    pt_cir_col = 0;
                else
                    pt_cir_col++;
            }
            else
            {
                pt_cir_col = pt_cir_col;
                pt_cir_row = pt_cir_row;
            }

            cv::Point pt_cir(pt_cir_col,pt_cir_row);
            cv::circle(matrix, pt_cir, 3, (0,0,0), -1);
        }

        void mv_line()
        {
            if(pt_line_row==0)
                pt_line_row = 499;
            else
                pt_line_row--;
        }

        int gameover()
        {
            if(pt_cir_row==pt_line_row)
                return -1;
            else
                return 0;
        }

        void show(cv::Mat matrix)
        {
            cv::Point pt_line1(0,pt_line_row), pt_line2(499,pt_line_row);
            cv::line(matrix,pt_line1, pt_line2, (0,0,0), 2);
            imshow("Moving Point",matrix);
        }
    Pnt_game()
    {
    }
    ~Pnt_game()
    {
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointer_game");
    ros::NodeHandle n;

    ros::Time cur_time = ros::Time::now();

    Pnt_game pnt;
        while(ros::ok())
        {
            ros::Duration dt;

            cv::Mat mat(500, 500,CV_8UC3,(255,255,255));
            int key = cv::waitKey(500);
            
            pnt.mv_pt(mat,key);
            
            dt = ros::Time::now() - cur_time;
            std::cout << dt << std::endl;
            if((int)dt.toSec() >= 1)
            {
                pnt.mv_line();
                // ros::Time cur_time = ros::Time::now();
                cur_time = ros::Time::now();
                dt = ros::Time::now() - cur_time;
            
                std::cout << "2" << std::endl;
            
            }

            pnt.show(mat);

            if(pnt.gameover()==-1)
            {
                break;
            }

            if(key==27)
            {
                break;
            }
        }
    
    

    return 0;
}