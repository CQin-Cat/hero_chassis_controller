//
// Created by qiayuan on 2/6/21.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PIDConfig.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>


namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    HeroChassisController() = default;
    ~HeroChassisController() override = default;

private:
    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;
    static double square(double x);
    void cb(const PIDConfig& config, uint32_t level);
    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
    void computeWheelEfforts(const ros::Time& time, const ros::Duration& period);
    void odom_update(const ros::Time& time, const ros::Duration& period);

    std::vector<hardware_interface::JointHandle> joint_handles_{};
    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
    ros::Subscriber cmd_vel_sub;
    std::shared_ptr<dynamic_reconfigure::Server<PIDConfig>> server; //服务器对象
    dynamic_reconfigure::Server<PIDConfig>::CallbackType cbType;
    tf::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub;

    double wheel_track, wheel_base, wheel_radius, rx, ry;
    double power_limit, effort_coeff_, velocity_coeff_, power_offset_;
    bool odomMode;
    double desired_front_left_velocity_ = 0.0;
    double desired_front_right_velocity_ = 0.0;
    double desired_back_left_velocity_ = 0.0;
    double desired_back_right_velocity_ =0.0;

    control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;
    double error_front_left_,error_front_right_,error_back_left_,error_back_right_;
    double cmd_effort_front_left_, cmd_effort_front_right_, cmd_effort_back_left_, cmd_effort_back_right_;

    double actual_front_left_velocity_ = 0.0;
    double actual_front_right_velocity_ = 0.0;
    double actual_back_left_velocity_ = 0.0;
    double actual_back_right_velocity_ = 0.0;

    double real_xvelocity = 0.0;
    double real_yvelocity = 0.0;
    double real_thvelocity =0.0;

    double dt = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dth = 0.0;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
};
}// namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
