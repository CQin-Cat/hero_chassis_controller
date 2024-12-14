
//
// Created by cqincat on 24-12-6.
//

#ifndef HERO_CHASSIS_CONTROLLER_CHASSIS_PID_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_CHASSIS_PID_CONTROLLER_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointControllerState.h>
#include <pluginlib/class_list_macros.h>
//#include <dynamic_reconfigure/server.h>
//#include <hero_chassis_controller/ChassisPidConfig.h>
#include <geometry_msgs/Twist.h>

namespace hero_chassis_controller {
    class ChassisPidController : public controller_interface::Controller<
        hardware_interface::EffortJointInterface> {
    public:
        ChassisPidController();

        ~ChassisPidController();

    bool init(hardware_interface::EffortJointInterface *robot,
              const std::string &joint_name, const control_toolbox::Pid &pid);
        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min,
                      const bool &antiwindup);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

        void printDebug();

        std::string getJointName();

        void setCommand(double &cmd);

        void getCommand(double &cmd);

        void starting(const ros::Time &time);

        void update(const ros::Time &time, const ros::Duration &period);

        void setCommandCB(const std_msgs::Float64ConstPtr &msg);

    private:
        control_toolbox::Pid pid_controller_;
        hardware_interface::JointHandle joint_;
        double command_;
        int loop_count_;
        double cmd;
        std::unique_ptr<
            realtime_tools::RealtimePublisher<
                control_msgs::JointControllerState> > controller_state_publisher_;
        ros::Subscriber sub_command_;
//    dynamic_reconfigure::Server<hero_chassis_controller::ChassisPidConfig> server_;
//    dynamic_reconfigure::Server<hero_chassis_controller::ChassisPidConfig>::CallbackType f_;

    };

    class EffortChassisController : public controller_interface::Controller<
        hardware_interface::EffortJointInterface> {
    public:
        EffortChassisController() = default;

        ~EffortChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;


    private:
        int state_{};
        ros::Time last_change_;

        ChassisPidController front_left_pid_controller_;
        ChassisPidController front_right_pid_controller_;
        ChassisPidController back_left_pid_controller_;
        ChassisPidController back_right_pid_controller_;

        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;

    };

//    class MecanumWheelController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
//    public:
//        MecanumWheelController();
//
//        ~MecanumWheelController();
//
//        bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &nh) override;
//
//        void update(const ros::Time &time, const ros::Duration &period) override;
//
//    private:
//        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
//
//        ros::Subscriber cmd_vel_sub_;
//        hardware_interface::JointHandle left_front_wheel_;
//        hardware_interface::JointHandle right_front_wheel_;
//        hardware_interface::JointHandle left_back_wheel_;
//        hardware_interface::JointHandle right_back_wheel_;
//
//        double wheel_base_;
//        double track_width_;
//        double wheel_radius_;
//    };
}
#endif //HERO_CHASSIS_CONTROLLER_CHASSIS_PID_CONTROLLER_H
