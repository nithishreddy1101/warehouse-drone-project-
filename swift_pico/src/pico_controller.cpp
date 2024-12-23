#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_array.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "pid_msg/msg/pid_tune.hpp"
#include "pid_msg/msg/pid_error.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct CMD
{
    int rc_roll;
    int rc_pitch;
    int rc_yaw;
    int rc_throttle;
    int rc_aux4;
};

struct ERROR
{
    float roll_error;
    float pitch_error;
    float throttle_error;
    float yaw_error;

};


class Swift_Pico : public rclcpp::Node
{
    public:
        Swift_Pico() : Node("pico_controller") //initializing ros node with name pico_controller
        {
            /* This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		     [x,y,z]*/
            drone_position[0] = 0.0;
            drone_position[1] = 0.0;
            drone_position[2] = 0.0;

            /* [x_setpoint, y_setpoint, z_setpoint]
            whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly*/
            setpoint[0] = 2;
            setpoint[1] = 2;
            setpoint[2] = 16;

            //Declaring a cmd of message type swift_msgs and initializing values
            cmd.rc_roll = 1500;
            cmd.rc_pitch = 1500;
            cmd.rc_yaw = 1500;
            cmd.rc_throttle = 1500;

            /*initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		    after tuning and computing corresponding PID parameters, change the parameters*/

            //proportional
            Kp[0] = 0;
            Kp[1] = 0;
            Kp[2] = 0;

            //integral
            Ki[0] = 0;
            Ki[1] = 0;
            Ki[2] = 0;

            //derivative
            Kd[0] = 0;
            Kd[1] = 0;
            Kd[2] = 0;


            //proportional
            Kp[0] = 0.02*451;
            Kp[1] = 0.02*451;
            Kp[2] = 0.02*501;

            //integral
            Ki[0] = 0.0001*3;
            Ki[1] = 0.0001*3;
            Ki[2] = 0.0001*35;

            //derivative
            Kd[0] = 0.6*806;
            Kd[1] = 0.6*806;
            Kd[2] = 0.6*830;
            /*-----------------------Add other required variables for pid here ----------------------------------------------*/
            max_values[0] =2000;
            max_values[1] =2000;
            max_values[2] =2000;

            min_values[0] =1000;
            min_values[1] =1000;
            min_values[2] =1000;

            error[0]=0.0;
            error[1]=0.0;
            error[2]=0.0;

            iterm[0]=0.0;
            iterm[1]=0.0;
            iterm[2]=0.0;

            diff[0]=0.0;
            diff[1]=0.0;
            diff[2]=0.0;

            previous_error[0]=0.0;
            previous_error[1]=0.0;
            previous_error[2]=0.0;
        
            sample_time = 60ms; //in milli-seconds

            //Publishing /drone_command, /throttle_error, /pitch_error, /roll_error
            command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
            pid_error_pub = this->create_publisher<pid_msg::msg::PIDError>("/pid_error", 10);


            //Add othher ROS 2 Publishers here

            //Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
            whycon_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/whycon/poses", 1, std::bind(&Swift_Pico::whycon_callback, this, _1));
            throttle_pid_sub = this->create_subscription<pid_msg::msg::PIDTune>("/throttle_pid", 1, std::bind(&Swift_Pico::altitude_set_pid, this, _1));
            roll_pid_sub = this->create_subscription<pid_msg::msg::PIDTune>("/roll_pid", 1, std::bind(&Swift_Pico::roll_set_pid, this, _1));
            pitch_pid_sub = this->create_subscription<pid_msg::msg::PIDTune>("/pitch_pid", 1, std::bind(&Swift_Pico::pitch_set_pid, this, _1));


            //------------------------Add other ROS 2 Subscribers here-----------------------------------------------------


            //Arming the drone
            arm();

            //Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(C++)
            timer_=this->create_wall_timer(
                60ms,std::bind(&Swift_Pico::pid,this));

            RCLCPP_INFO(this->get_logger(), "Swift Pico Started");




        }

    private:

        //declare all the variables, arrays, strcuts etc. here
        float drone_position[3];
        int setpoint[3];
        CMD cmd;
        ERROR pid_error;
        int max_values[3];
        int min_values[3];
        float error[3];
        float iterm[3];
        float diff[3];
        float previous_error[3];

        std::chrono::milliseconds sample_time;
	auto cmd = swift_msgs::msg::SwiftMsgs();
        auto error_pub = pid_msg::msg::PIDError();
        float Kp[3];
        float Ki[3];
        float Kd[3];

        //declare the publishers and subscribers here
        rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub;
        rclcpp::Publisher<pid_msg::msg::PIDError>::SharedPtr pid_error_pub;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr whycon_sub;
        rclcpp::Subscription<pid_msg::msg::PIDTune>::SharedPtr throttle_pid_sub;
        rclcpp::Subscription<pid_msg::msg::PIDTune>::SharedPtr roll_pid_sub;
        rclcpp::Subscription<pid_msg::msg::PIDTune>::SharedPtr pitch_pid_sub;

        rclcpp::TimerBase::SharedPtr timer_;

        //define functions and callbacks here

        void disarm()
        {
            auto cmd = swift_msgs::msg::SwiftMsgs();
            cmd.rc_roll = 1000;
            cmd.rc_pitch = 1000;
            cmd.rc_yaw = 1000;
            cmd.rc_throttle = 1000;
            cmd.rc_aux4 = 1000;
            command_pub->publish(cmd);
        }

        void arm()
        {   
            auto cmd = swift_msgs::msg::SwiftMsgs();
            disarm();
            cmd.rc_roll = 1500;
            cmd.rc_pitch = 1500;
            cmd.rc_yaw = 1500;
            cmd.rc_throttle = 1500;
            cmd.rc_aux4 = 2000;
            command_pub->publish(cmd);
        }

        void whycon_callback(const geometry_msgs::msg::PoseArray & msg)
        {
            drone_position[0] = msg.poses[0].position.x;
            drone_position[1] = msg.poses[0].position.y;
            drone_position[2] = msg.poses[0].position.z;
        }
	
        void altitude_set_pid(const pid_msg::msg::PIDTune & alt)
        {
            Kp[2] = alt.kp * 0.02;
		    Ki[2] = alt.ki * 0.0001;
		    Kd[2] = alt.kd * 0.6;
        }

        void roll_set_pid(const pid_msg::msg::PIDTune & alt)
        {
            Kp[0] = alt.kp * 0.02; 
		    Ki[0] = alt.ki * 0.0001;
		    Kd[0] = alt.kd * 0.6;
        }

        void pitch_set_pid(const pid_msg::msg::PIDTune & alt)
        {
            Kp[1] = alt.kp * 0.02; 
		    Ki[1] = alt.ki * 0.0001;
		    Kd[1] = alt.kd * 0.6;
        }



        void pid()
        {
            error[0]=drone_position[0]-setpoint[0];
            error[1]=drone_position[1]-setpoint[1];
            error[2]=drone_position[2]-setpoint[2];

            iterm[0]=iterm[0]+error[0];
            iterm[1]=iterm[1]+error[1];
            iterm[2]=iterm[2]+error[2];

            diff[0]=error[0]-previous_error[0];
            diff[1]=error[1]-previous_error[1];
            diff[2]=error[2]-previous_error[2];


            float roll_out= Kp[0]*error[0]+Ki[0]*iterm[0]+Kd[0]*diff[0];

            float pitch_out= Kp[1]*error[1]+Ki[1]*iterm[1]+Kd[1]*diff[1];

            float throttle_out= Kp[2]*error[2]+Ki[2]*iterm[2]+Kd[2]*diff[2];

            cmd.rc_throttle = int(1500+throttle_out);
            cmd.rc_roll = int(1500-roll_out);
            cmd.rc_pitch = int(1500 + pitch_out);

            /*std::string logger_= "Roll: "+std::to_string(roll_out)+" Pitch: "+ std::to_string(pitch_out)+" throttle: "+std::to_string(throttle_out);

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", logger_.c_str());*/


            previous_error[0]=error[0];
            previous_error[1]=error[1];
            previous_error[2]=error[2];

            command_pub->publish(cmd);

            error_pub.roll_error=error[0];
            error_pub.pitch_error=error[1];
            error_pub.throttle_error=error[2]-3.0;
            pid_error_pub->publish(error_pub);

        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Swift_Pico>());
    rclcpp::shutdown();
    return 0;
}
