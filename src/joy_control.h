#ifndef JOY_CONTROL_H
#define JOY_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <iostream>

namespace joy_control
{
    class control_Joy
    {
    public:
        void callback_joy(const sensor_msgs::Joy::ConstPtr& msg);
        void callback_speed(const std_msgs::Float64::ConstPtr& msg);
        void callback_steering(const std_msgs::Float64::ConstPtr& msg);

        const int ON = 1;
        const int OFF = 0;

        ///////// for ros melodic joy package /////////
        // ps4 dualshock joystick button mapping
        // const int X = 1;
        // const int O = 2;
        // const int triangle = 3;
        // const int square = 0;
        // const int L1_Button = 4;
        // const int R1_Button = 5;
        // const int L2_Trigger = 6;
        // const int R2_Trigger = 7;
        // const int SHARE_button = 8;
        // const int Option_button = 9;
        // const int PS_button = 12;
        // const int Button_stick_Left_push = 10;
        // const int Button_stick_Right_push = 11;

        // // ps4 dualshock joystick axis mapping
        // const double Left_stick_Left_and_Right = 0;
        // const double Left_stick_Up_and_Down = 1;
        // const double Left_trigger = 3;
        // const double Right_stick_Left_and_Right = 2;
        // const double Right_stick_Up_and_Down = 5;
        // const double Right_trigger = 4;

        ///////// for ros kinetic joy package /////////
        // ps4 dualshock joystick button mapping
        const int X = 0;
        const int O = 1;
        const int triangle = 2;
        const int square = 3;
        const int L1_Button = 4;
        const int R1_Button = 5;
        const int L2_Trigger = 6;
        const int R2_Trigger = 7;
        const int SHARE_button = 8;
        const int Option_button = 9;
        const int PS_button = 10;
        const int Button_stick_Left_push = 11;
        const int Button_stick_Right_push = 12;

        // ps4 dualshock joystick axis mapping
        const double Left_stick_Left_and_Right = 0;
        const double Left_stick_Up_and_Down = 1;
        const double Left_trigger = 2;
        const double Right_stick_Left_and_Right = 3;
        const double Right_stick_Up_and_Down = 4;
        const double Right_trigger = 5;

        control_Joy();
        ~control_Joy();

        std_msgs::Float32 m_Scooter_Speed;
        dynamixel_sdk_examples::SetPosition m_Scooter_Steering;

        ros::Publisher pub_load_path;
        ros::Publisher pub_tracking;
        ros::Publisher pub_make_path;
        ros::Publisher pub_gps_init;
        std_msgs::String tmp_msg;

        float max_speed;
        float max_steering; 

        void run();

    private:
        void speed_control();
        void steering_control();

        int m_start;
        int m_make_path;
        int m_load_path;
        int m_tracking;
        int m_gps_init;

        double m_linear_x_stick;
        double m_angular_z_stick;

        bool m_bFlagStart;
        bool m_bFlagMakePath;
        bool m_bFlagTracking;

        float m_dLinear_x_vel;
	    float m_d_steer_angle;

        int m_L1_button;
    };
}

#endif