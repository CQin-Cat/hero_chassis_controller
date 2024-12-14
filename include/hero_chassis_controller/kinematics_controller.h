//
// Created by cqincat on 24-12-14.
//

#ifndef ROS1_KINEMATICIS_CONTROLLER_H
#define ROS1_KINEMATICIS_CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <tf/transform_broadcaster.h>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller{
    class KinematicsController: public controller_interface::Controller<
        hardware_interface::VelocityJointInterface> {
    public:
        KinematicsController();
        ~KinematicsController();

        bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle& nh);
        void update(const ros::Time& time, const ros::Duration& period) override;
    private:
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);//订阅信息
        void computeWheelVelocities(double vx, double vy, double omega);//计算各轮速度
        void computeBaseVelocityFromWheels();//计算底盘线速度和角速度
        void updateOdometry(const ros::Time& time);//更新里程数据

        double wheel_radius_;//半径
        double wheel_base_;//轴距
        double wheel_track_;//轮宽
        double cmd_vel_[4];//期望速度
        double x_, y_, theta_; // 底盘的 x, y 坐标和朝向
        double linear_velocity_, angular_velocity_;//线速度、角速度

        hardware_interface::JointHandle left_front_wheel_joint_;
        hardware_interface::JointHandle right_front_wheel_joint_;
        hardware_interface::JointHandle left_back_wheel_joint_;
        hardware_interface::JointHandle right_back_wheel_joint_;

        ros::Subscriber cmd_vel_sub_;
        ros::Publisher odom_pub_;
        tf::TransformBroadcaster tf_broadcaster_;
        nav_msgs::Odometry odom_;

        ros::Time last_time_;


    };
}
#endif //ROS1_KINEMATICIS_CONTROLLER_H
