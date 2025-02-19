#include <windmill_controller.h>
#include <memory>
#include <pluginlib/class_list_macros.hpp>

namespace windmill_controller{
     WindMillController::WindMillController() : t_(0){}
     WindMillController::~WindMillController(){}
    bool WindMillController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        effort_ = controller_nh.subscribe("/joint_states",1000,&WindMillController::effortback,this);
        pid_controller_.init(ros::NodeHandle(controller_nh,"pid_control"));
        controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>>(
                controller_nh, "state", 1);
        cmd_sub_ = root_nh.subscribe("/cmd_vel",1000,&WindMillController::cmdCallback,this);
        sub_command_ = controller_nh.subscribe<std_msgs::Float64>("command", 1, &WindMillController::setCommandCB, this);
        windmill_joint_ = effort_joint_interface->getHandle("joint1");
        return true;
    }
    void WindMillController::getGains(double &p, double &i, double &d, double &i_max, double &i_min){
        pid_controller_.getGains(p,i,d,i_max,i_min);
    }
    void WindMillController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup){
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }
    void WindMillController::update(const ros::Time& time, const ros::Duration& period){
        ros::param::get("windmill_mode",windmill_mode_);
        if(windmill_mode_==0){
            //初始模式（小）
            target_vel_=10.0;
            command_ = target_vel_;
            double windmill_error = command_ - windmill_joint_.getVelocity();
            double windmill_effort = pid_controller_.computeCommand(windmill_error,period);
            windmill_joint_.setCommand(windmill_effort);
        }
        else if(windmill_mode_ == 1){
            //BIG
            windmill_mode_ = 2;
            t_ = 0;
            srand((unsigned)std::time(NULL));
            a_ = double((rand()%(1045-780))+780)/1000;
            w_ = double((rand()%(2000-1884))+1884)/1000;
            target_vel_ = a_*sin(w_*t_)+(2.090-a_);
            command_ = target_vel_;
            double windmill_error = command_ - windmill_joint_.getVelocity();
            double windmill_effort = pid_controller_.computeCommand(windmill_error,period);
            windmill_joint_.setCommand(windmill_effort);
            t_start_ = ros::Time::now().toSec();
        }
        else if(windmill_mode_ ==2){
            target_vel_ = a_*sin(w_*t_)+(2.090-a_);
            command_ = target_vel_;
            t_ = ros::Time::now().toSec() - t_start_;
            double windmill_error = command_ - windmill_joint_.getVelocity();
            double windmill_effort = pid_controller_.computeCommand(windmill_error,period);
            windmill_joint_.setCommand(windmill_effort);
        }


    }
    void WindMillController::setCommandCB(const std_msgs::Float64ConstPtr& state)
    {
        state_ = state->data;
    }
    void WindMillController::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        geometry_msgs::Twist vel;
        vel.linear.x=msg->linear.x;
        vel.linear.y=msg->linear.y;
        vel.angular.z=msg->angular.z;
        }

}
    PLUGINLIB_EXPORT_CLASS(windmill_controller::WindMillController,controller_interface::ControllerBase)

