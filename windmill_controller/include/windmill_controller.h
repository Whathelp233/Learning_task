#ifndef WINDMILL_CONTROLLER_H
#define WINDMILL_CONTROLLER_H
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <cstdlib>
#include <time.h>
namespace windmill_controller {
    class WindMillController  : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
    public:
        WindMillController();
        ~WindMillController()  override;
        bool init(hardware_interface::EffortJointInterface* effort_joint_interface,ros::NodeHandle& root_nh
        ,ros::NodeHandle& controller_nh) override;
        void update(const ros::Time& time, const ros::Duration& period) override;
        hardware_interface::JointHandle windmill_joint_;
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        double command_ =0.0;
        double kf_ = 0.0;
        double vel_;
        void effortback(const sensor_msgs::JointState::ConstPtr &msg);
        double joint_torque_[100];
        double joint_Velocity_[100];
        double joint_effort_,k1_,k2_;
        int windmill_mode_;
        double target_vel_,a_,t_,b_,w_,t_start_;
    private:
        int state_{};
        ros::Time last_change_;
        control_toolbox::Pid pid_controller_;
        std::unique_ptr<
                realtime_tools::RealtimePublisher<
                        control_msgs::JointControllerState> > controller_state_publisher_ ;
        ros::Subscriber sub_command_;
        ros::Subscriber effort_;
        int loop_count_ = 0;
        ros::Subscriber cmd_sub_;
        void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void setCommandCB(const std_msgs::Float64ConstPtr& state);
    };
}
#endif
