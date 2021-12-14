#include "joy_control.h"

using namespace joy_control;

using namespace dynamixel;

control_Joy::control_Joy()
{

    m_start = 0;
    m_make_path = 0;
    m_load_path = 0;
    m_tracking = 0;
    m_gps_init = 0;

    m_linear_x_stick = 0.0;
    m_angular_z_stick = 0.0;

    m_bFlagStart = true;
    m_bFlagMakePath = false;
    m_bFlagTracking = false;
   
    m_dLinear_x_vel = 0.0;
    m_d_steer_angle = 0.0;


    std::stringstream ss;
    ss << "hello";
    tmp_msg.data = ss.str();

    m_L1_button = 0;
}

control_Joy::~control_Joy()
{    
}

void control_Joy::callback_joy(const sensor_msgs::Joy::ConstPtr& msg)
{
    m_make_path = msg->buttons[square];
    m_load_path = msg->buttons[triangle];
    m_tracking = msg->buttons[O];
    m_gps_init = msg->buttons[X];
    m_start = msg->buttons[PS_button];
    m_L1_button = msg->buttons[L1_Button];

    m_linear_x_stick = msg->axes[Left_stick_Up_and_Down];
    m_angular_z_stick = msg->axes[Right_stick_Left_and_Right];

    if(m_make_path == ON && m_bFlagMakePath == false){
        pub_make_path.publish(tmp_msg);
        m_bFlagMakePath = true;
        std::cout << "*** path making ***" << std::endl;
    }
    else if(m_make_path == ON && m_bFlagMakePath == true){
        pub_make_path.publish(tmp_msg);
        m_bFlagMakePath = false;
        std::cout << "*** path making is done ***" << std::endl;
    }
    if(m_load_path == ON){
        pub_load_path.publish(tmp_msg);
        std::cout << "*** load path ***" << std::endl;
    }
    if(m_tracking == ON && m_bFlagTracking == false){
        pub_tracking.publish(tmp_msg);
        m_bFlagTracking = true;
        std::cout << "*** start tracking ***" << std::endl;
    }
    else if(m_tracking == ON && m_bFlagTracking == true){
        pub_tracking.publish(tmp_msg);
        m_bFlagTracking = false;
        std::cout << "*** stop tracking ***" << std::endl;
    }
    if(m_gps_init == ON){
        pub_gps_init.publish(tmp_msg);
        std::cout << "*** gps init ***" << std::endl;
    }

    if(m_start == ON && m_bFlagStart == false) {
        m_bFlagStart = true;
        std::cout << "*** START manual control ***" << std::endl;
    }
    else if (m_start == ON && m_bFlagStart == true){
        m_bFlagStart = false;
        std::cout << "*** Manual control OFF ***" << std::endl;
    }

    // if(m_bFlagStart == true){
    //     speed_control();
    //     steering_control();

    //     m_Scooter_Speed.data = m_dLinear_x_vel;
    //     m_Scooter_Steering.id = 1;
    //     m_Scooter_Steering.position = (int)m_d_steer_angle;
    // }
    // else{
    //     //m_Scooter_Speed.data = 0.0;
    //     m_Scooter_Speed.data = m_dLinear_x_vel;
    //     m_Scooter_Steering.id = 1;
    //     m_Scooter_Steering.position = (int)m_d_steer_angle * 3000;
    // }    
}

void control_Joy::callback_speed(const std_msgs::Float64::ConstPtr& msg)
{
    m_dLinear_x_vel = (float)msg->data;
}

void control_Joy::callback_steering(const std_msgs::Float64::ConstPtr& msg)
{
    m_d_steer_angle = (float)msg->data;
    if (m_d_steer_angle > 20.0) {
        m_d_steer_angle = 20.0;
    }
    else if (m_d_steer_angle < -20.0){
        m_d_steer_angle = -20.0;
    }
}

void control_Joy::speed_control()
{
    m_dLinear_x_vel = m_linear_x_stick * max_speed;
    if(m_dLinear_x_vel < 0)
    {
        m_dLinear_x_vel = 0.0;
    }
}

void control_Joy::steering_control()
{
    m_d_steer_angle = m_angular_z_stick * 3000.0 * max_steering;
}

void control_Joy::run()
{
    if(m_bFlagStart == true){
        speed_control();
        steering_control();

        m_Scooter_Speed.data = m_dLinear_x_vel;
        m_Scooter_Steering.id = 1;
        m_Scooter_Steering.position = (int)m_d_steer_angle;
    }
    else{
        if(m_L1_button == ON){
            m_Scooter_Speed.data = m_dLinear_x_vel + m_linear_x_stick * max_speed;
            if(m_Scooter_Speed.data > max_speed){
                m_Scooter_Speed.data = max_speed;
            }
            else if(m_Scooter_Speed.data < 0){
                m_Scooter_Speed.data = 0 ;
            }
            m_Scooter_Steering.id = 1;
            m_Scooter_Steering.position = (int)m_d_steer_angle * 3000 + m_angular_z_stick * 3000.0 * max_steering;
            if (m_Scooter_Steering.position > 60000){
                m_Scooter_Steering.position = 60000;
            }
            else if(m_Scooter_Steering.position < -60000){
                m_Scooter_Steering.position = -60000;
            }
        }
        else {
            m_Scooter_Speed.data = m_dLinear_x_vel;
            m_Scooter_Steering.id = 1;
            m_Scooter_Steering.position = (int)m_d_steer_angle * 3000;
        }
    }    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_control");
    ros::NodeHandle nh;

    control_Joy c_joy;

    ros::Subscriber sub_Joy = nh.subscribe<sensor_msgs::Joy>("joy", 100, &control_Joy::callback_joy, &c_joy);
    ros::Subscriber sub_cmd_scooter_speed = nh.subscribe<std_msgs::Float64>("/cmd_scooter_speed", 10, &control_Joy::callback_speed, &c_joy);
    ros::Subscriber sub_cmd_scooter_steering = nh.subscribe<std_msgs::Float64>("/cmd_scooter_steering", 10, &control_Joy::callback_steering, &c_joy);

    ros::Publisher pub_velocity = nh.advertise<std_msgs::Float32>("/velocity", 10);
    ros::Publisher pub_steering = nh.advertise<dynamixel_sdk_examples::SetPosition>("/set_position", 10);
    c_joy.pub_make_path = nh.advertise<std_msgs::String>("/make_path", 10);
    c_joy.pub_load_path = nh.advertise<std_msgs::String>("/load_path", 10);
    c_joy.pub_tracking = nh.advertise<std_msgs::String>("/tracking", 10);
    c_joy.pub_gps_init = nh.advertise<std_msgs::String>("/init", 10);

    nh.getParam("/max_speed", c_joy.max_speed);
    nh.getParam("/max_steering", c_joy.max_steering);
    
    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        c_joy.run();

        pub_velocity.publish(c_joy.m_Scooter_Speed);
        pub_steering.publish(c_joy.m_Scooter_Steering);

        loop_rate.sleep();
    }

    return 0;    
}