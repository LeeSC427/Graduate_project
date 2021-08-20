#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <mutex>
#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
// #include "connection/joy.h"

#define PI 3.1415926563
#define T_RPM 20
#define R_RPM 20
#define R_RADIAN 2
#define Degree 30

class PORT
{
    public:
        std::string port_name = "/dev/ttyUSB0";
        std::string baudrate = "19200";
};

class MOTOR_INFO
{
    public:
        short L_rpm = 0;
        short R_rpm = 0;
};

class CONNECTION
{
    public:
        PORT port;
        MOTOR_INFO motor_info;
        int fd;
        bool tqoff;
        int read_buf_pos = 0;
        int write_buf_pos = 0;
        unsigned char read_buf[256];
        unsigned char write_buf[256];
        std::mutex fd_mtx;
        std::chrono::system_clock::time_point prev_read_time;
        int val = 0;
        // int t_rpm = 100;
        int r_rpm = 20;

    bool CONNECT()
    {
        fd = 0;
        struct termios newtio;

        while(ros::ok())
        {
            fd = open(port.port_name.c_str(), O_RDWR | O_NOCTTY);

            if(fd < 0)
            {
                ROS_ERROR("Not connected, check your USB port.");
            }
            else
                break;
        }

        ROS_INFO("RObot connected!", port.port_name.c_str());

        memset(&newtio, 0, sizeof(newtio));

        newtio.c_cflag = B19200;
        newtio.c_cflag |= CS8;
        newtio.c_cflag |= CLOCAL;
        newtio.c_cflag |= CREAD;
        newtio.c_iflag = 0;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;
        newtio.c_cflag |= CS8;

        tcflush(fd, TCIFLUSH);

        tcsetattr(fd, TCSANOW, &newtio);

        // ros::Rate rate(15);
        // while(ros::ok())
        // {
        //     receive_task();
        //     send_task(joy_FB, joy_LR, joy_ACT);
        //     rate.sleep();
        // }

        return 1;
    }

    bool write_serial(unsigned char* data, int size)
    {
        // ROS_INFO("write_serial");

        if(size == 0 | fd <= 0) return 0;
        bool check_write = false;

        // fd_mtx.lock();
        
        val = write(fd, data, size);
        
        if(val == size)
            check_write = true;

        // fd_mtx.unlock();

        return check_write;
    }

    void send_task(float T_vel, float R_vel, int joy_ACT)
    {
        // ROS_INFO("send_task");
        run_motor(T_vel, R_vel, joy_ACT);
    }

    short run_motor(float T_vel, float R_vel, int joy_ACT)
    {
        unsigned char l_rpm = 0;
        unsigned char r_rpm = 0;
        float left_wheel = 0, right_wheel = 0;
        int rpm_1 = 0;
        int rpm_2 = 0;
        if(fd <= 0)
        {
            ROS_WARN("write2motor fd: %d", fd);

            return 0;
        }

        unsigned char write_motor_cmd[13] = {183, 184, 1, 207, 7, 1, 0, 0, 1, 0, 0, 0, 184};

        // ROS_INFO("joy_FB: %lf", joy_ACT);

        if(joy_ACT != 0)
        {
            // ROS_INFO("joy_ACT: %d",joy_ACT);
            // float rad = getRadian(R_vel * Degree);
            // float Radian = rad * R_RADIAN;
            // float min_Radian = PI - Radian;
            // ROS_INFO("cos-0.5sin: %lf", cos(Radian)-0.5*sin(Radian));

            // if(T_vel >= 0)
            // {
            //     left_wheel = (-1)*(T_RPM * T_vel + R_RPM * R_vel) * (cos(Radian) - 0.5 * sin(Radian));
            //     right_wheel = (T_RPM * T_vel + R_RPM * R_vel) * (cos(Radian) + 0.5 * sin(Radian));
            // }
            // else if(T_vel < 0)
            // {
            //     left_wheel = (-1)*(T_RPM * T_vel + R_RPM * R_vel) * (cos(min_Radian) - 0.5 * sin(min_Radian));
            //     right_wheel = (T_RPM * T_vel + R_RPM * R_vel) * (cos(min_Radian) + 0.5 * sin(min_Radian));
            // }

            // if(left_wheel > 0)
            // {
            //     l_rpm = 256 - left_wheel;
            //     write_motor_cmd[7] = 255;
            // }
            // else if(left_wheel < 0)
            // {
            //     l_rpm = left_wheel;
            //     write_motor_cmd[7] = 0;
            // }

            // if(right_wheel > 0)
            // {
            //     r_rpm = 256 - right_wheel;
            //     write_motor_cmd[10] = 255;  
            // }
            // else if(right_wheel < 0)
            // {
            //     r_rpm = right_wheel;
            //     write_motor_cmd[10] = 0;
            // }
        
            // write_motor_cmd[6] = l_rpm;
            // write_motor_cmd[9] = r_rpm;
            
            rpm_1 = T_RPM * (T_vel + R_vel);
            rpm_2 = T_RPM * (T_vel - R_vel);

        ROS_INFO("rpm_1: %d, rpm_2: %d",rpm_1, rpm_2);
            

            if(rpm_1 > 0)
            {
                r_rpm = rpm_1;
                write_motor_cmd[9] = r_rpm;
                write_motor_cmd[10] = 0;
            }
            else if(rpm_1 < 0)
            {
                r_rpm = 256 - abs(rpm_1);
                write_motor_cmd[9] = r_rpm;
                write_motor_cmd[10] = 255;
            }
            if(rpm_2 > 0)
            {
                l_rpm = 256 - rpm_2;
                write_motor_cmd[6] = l_rpm;
                write_motor_cmd[7] = 255;

            }
            else if(rpm_2 < 0)
            {
                l_rpm = abs(rpm_2);
                write_motor_cmd[6] = l_rpm;
                write_motor_cmd[7] = 0;
            }
        }


        unsigned char chk = check_sum_send(&write_motor_cmd[0],12);
        write_motor_cmd[12] = chk;

        // ROS_INFO("left_wheel_val: %d", write_motor_cmd[6]);

        write_speed_serial(write_motor_cmd, 13);
    }

    bool write_speed_serial(unsigned char* data, int size)
    {
        if(size == 0 | fd <= 0)
        return 0;

        bool check_write = false;

        // fd_mtx.lock();
        int val = write(fd, data, size);
        if(val == size)
        {
            check_write = true;
        }   
        // fd_mtx.unlock();

        return check_write;
    }

    void receive_task()
    {
        // ROS_INFO("receive task");
            read_motor_state();
    }

    short read_motor_state(void)
    {

        // ROS_INFO("read motor speed");
        if(fd <= 0)
        {
            ROS_WARN("read_motor_state fd: %d", fd);
            
            return 0;
        }
        
        request_motor_state();

        std::chrono::system_clock::time_point read_time = std::chrono::system_clock::now();
        std::chrono::duration<double> dt_duration = read_time - prev_read_time;

        prev_read_time = read_time;

        double dt = dt_duration.count();

        int read_size = read(fd, &read_buf[read_buf_pos], 128);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // ROS_INFO("read_size: %d", read_size);
        // ROS_INFO("fd: %d", fd);
    
        read_buf_pos += read_size;


        // ROS_INFO("read_buf_pos: %d", read_buf_pos);
        

        if(read_size <= 0)
            return 0;

        unsigned char read_id[5] = {184, 183, 1, 210, 18};

        for(int i = 0; i < read_buf_pos - 23; i++)
        {
            if(read_buf[i] == read_id[0])
            if(read_buf[i+1] == read_id[1])
            if(read_buf[i+2] == read_id[2])
            if(read_buf[i+3] == read_id[3])
            if(read_buf[i+4] == read_id[4])
            {
                int R_sign = -1, L_sign = -1;

                motor_info.R_rpm = (short)R_sign * Byte2Short(read_buf[i+5], read_buf[i+6]);
                motor_info.L_rpm = (short)L_sign * Byte2Short(read_buf[i+14], read_buf[i+15]);

                unsigned char chk = check_sum_send(&read_buf[i], 23);
                
                // ROS_INFO("chk = %d", chk);
                // ROS_INFO("r_buf = %d", read_buf[i+23]);

                if(read_buf[i+23] != chk)
                {
                    ROS_WARN("checksum error");

                    continue;
                }
                else
                {
                    // ROS_INFO("R_rpm = %d", motor_info.R_rpm);
                    // ROS_INFO("L_rpm = %d", motor_info.L_rpm);
                }
            } 
        }
    }

    short Byte2Short(unsigned char low, unsigned char high)
    {
        return ((short) low | (short) high << 8);
    }

    void request_motor_state()
    {
    
        // ROS_INFO("request motor state");
        unsigned char read_motor_cmd[7] = {183, 184, 1, 4, 1, 210, 185};
        unsigned char chk = check_sum_send(&read_motor_cmd[0], 6);
        // ROS_INFO("chk: %i", chk);
        write_serial(read_motor_cmd,7);
        
    }

    unsigned char check_sum_send(unsigned char* Array, int size)
    {
        int byTmp = 0;
        short i;

        for(i = 0; i < size; i++)
        {
            byTmp += *(Array + i);
        }

        return (~byTmp + 1);
    }

    unsigned char check_sum_receive(unsigned char* Array, int size)
    {
        short i;
        int cbySum = 0;

        for(i = 0; i < size; i++)
        {
            cbySum += *(Array + i);
        }
        if(cbySum == 0)
            return 1;
        else
            return 0;
    }

    float getRadian(float degree)
    {
        return degree * (PI / 180);
    }

};
