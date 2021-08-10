#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <iostream>

class Pnt
{
    public:
    
    // cv::Scalar white(255,255,255);
    // cv::Scalar black(0,0,0);

        int pt_col = 249;
        int pt_row = 249;

        void mv_pt(int input)
        {
            cv::Mat mat(500,500,CV_8UC3,(255,255,255));

            if(input==119) //w
            {
                if(pt_row == 0)
                    pt_row = 499;
                else
                    pt_row--;
            }
            
            else if(input==97) //a
            {
                if(pt_col == 0)
                    pt_col = 499;
                else
                    pt_col--;
            }

            else if(input==115)   //s
            {
                if(pt_row == 499)
                    pt_row = 0;
                else
                    pt_row++;
            }

            else if(input==100)
            {
                if(pt_col == 499)
                    pt_col = 0;
                else
                    pt_col++;
            }

            cv::Point pt(pt_col,pt_row);
            cv::circle(mat, pt, 5, (0,0,0), -1);

            imshow("Moving Point",mat);
        }

    Pnt()
    {
    }
    ~Pnt()
    {
    }
};


int main(int argc, char **argv)
{
    
    Pnt pnt;
    
    while(1)
    {
        int key = cv::waitKey(500);
        
        pnt.mv_pt(key);

        if(key==27)
        {
            break;
        }
    
    }

    return 0;
}