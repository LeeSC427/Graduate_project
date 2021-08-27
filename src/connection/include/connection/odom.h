#include <iostream>
#include "ros/ros.h"
#include <cmath>

#define _USE_MATH_DEFINES

class Values
{
    public:
        int r_rpm;
        int l_rpm;
        double wheel_dist;
        double dt;
    
    Values()
    {
                r_rpm = 0;
                l_rpm = 0;
                dt = 0.0;
                wheel_dist = 0.0;

    }

    ~Values()
    {
    }
};

// class Odom
// {   
//     public:
//         double x_prev;
//         double y_prev;
//         double x_location;
//         double y_location;
//         double angle_prev;
//         double angle;

//     Odom()
//     {
//                 x_prev = 0.0;
//                 y_prev = 0.0;
//                 x_location = 0.0;
//                 y_location = 0.0;
//                 angle_prev = 0.0;
//                 angle = 0.0;

//     }

//     ~Odom()
//     {
//     }
// };

class odometry
{
    public:
        // Odom odom;
        Values val;

        double x_prev = 0.0;
        double y_prev = 0.0;
        double x_location = 0.0;
        double y_location = 0.0;
        double angle_prev = 0.0;
        double angle = 0.0;

        void get_val(short R_rpm, short L_rpm, double Wheel_dist, double Dt)
        {
            // val.r_rpm = r_rpm;
            // val.l_rpm = l_rpm;
            // val.wheel_dist = wheel_dist;
            // val.dt = dt;
            val.r_rpm = (int) R_rpm;
            val.l_rpm = (int) L_rpm;
            val.wheel_dist = Wheel_dist;
            val.dt = Dt;
        }
        void get_odom()
        {
            if(val.dt > 0.001){
                double vr = ((double)val.r_rpm / 60.0) * 0.13 * M_PI; // m/sec
                double vl = ((double)val.l_rpm / 60.0) * 0.13 * M_PI; // m/sec
                double V = (vr + vl) / 2.0;
                double w = (vr - vl) / val.wheel_dist;
                std::cout << val.dt << std::endl;
                std::cout << vr << " " << vl << " " << V << " " << w << " " << std::endl;
                std::cout << angle_prev << " " << x_prev << " " << y_prev << std::endl;

                angle = angle_prev + w * val.dt;
                x_location = x_prev + V * cos(angle_prev) * val.dt;
                y_location = y_prev + V * sin(angle_prev) * val.dt;
                
                
                if(angle > 2.0 * 3.14)
                {
                    angle_prev = angle - 2.0 * 3.14;
                }
                else if(angle < 0.0)
                {
                    angle_prev = angle + 2.0 * 3.14;
                }
                std::cout << angle_prev << std::endl;

                x_prev = x_location;
                y_prev = y_location;
            }
        }

        void OD(short r_rpm, short l_rpm, double wheel_dist, double dt)
        {
            get_val(r_rpm, l_rpm, wheel_dist, dt);
            get_odom();
        }

        odometry()
        {
        }

        ~odometry()
        {
        }

};