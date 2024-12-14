//
// Created by cqincat on 24-12-14.
//
#include "hero_chassis_controller/kinematics_controller.h"

namespace hero_chassis_controller{

    bool KinematicsController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
        if (!nh.getParam("wheel_radius", wheel_radius_) ||
            !nh.getParam("wheel_base", wheel_base_) ||
            !nh.getParam("wheel_track", wheel_track_)) {
            ROS_ERROR("KinematicsController: Missing required parameters (wheel_radius, wheel_base, wheel_track)");
            return false;
        }//从服务器获取参数
        try {
            left_front_wheel_joint_ = hw->getHandle("hero::left_front_wheel_joint");
            right_front_wheel_joint_ = hw->getHandle("hero::right_front_wheel_joint");
            left_back_wheel_joint_ = hw->getHandle("hero::left_back_wheel_joint");
            right_back_wheel_joint_ = hw->getHandle("hero::right_back_wheel_joint");
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM("KinematicsController: " << e.what());
            return false;
        }//用心感受这个try_catch

        cmd_vel_[4] = {0.0};
        cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &KinematicsController::cmdVelCallback, this);//准备接收速度信息的

        x_ = y_ = theta_ = 0.0;
        linear_velocity_ = angular_velocity_ = 0.0;
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);//准备解算

        last_time_ = ros::Time::now();

        return true;
    }
    void KinematicsController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
            computeWheelVelocities(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void KinematicsController::computeWheelVelocities(double vx, double vy, double omega) {
            double half_base = wheel_base_ / 2.0;
            double half_track = wheel_track_ / 2.0;

            cmd_vel_[0] = (vx - vy - omega * (half_base + half_track)) / wheel_radius_;  // 左前轮
            cmd_vel_[1] = (vx + vy + omega * (half_base + half_track)) / wheel_radius_;  // 右前轮
            cmd_vel_[2] = (vx + vy - omega * (half_base + half_track)) / wheel_radius_;  // 左后轮
            cmd_vel_[3] = (vx - vy + omega * (half_base + half_track)) / wheel_radius_;  // 右后轮
    }

    void KinematicsController::computeBaseVelocityFromWheels() {
        double vlf = left_front_wheel_joint_.getVelocity();
        double vrf = right_front_wheel_joint_.getVelocity();
        double vlb = left_back_wheel_joint_.getVelocity();
        double vrb = right_back_wheel_joint_.getVelocity();//获取轮子速度

        double vx = wheel_radius_ / 4.0 * (vlf + vrf + vlb + vrb);
        double vy = wheel_radius_ / 4.0 * (-vlf + vrf + vlb - vrb);
        double omega = wheel_radius_ / (4.0 * (wheel_base_ + wheel_track_)) * (-vlf + vrf - vlb + vrb);//计算线速度和角速度

        linear_velocity_ = vx;
        angular_velocity_ = omega;
    }


    void KinematicsController::updateOdometry(const ros::Time& time) {

        ros::Duration dt = time - last_time_;
        last_time_ = time;

        // 计算增量
        double delta_x = linear_velocity_ * cos(theta_) * dt.toSec();
        double delta_y = linear_velocity_ * sin(theta_) * dt.toSec();
        double delta_theta = angular_velocity_ * dt.toSec();

        // 更新里程计状态
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // 发布里程计消息
        odom_.header.stamp = time;
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";

        // 位置
        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;
        odom_.pose.pose.position.z = 0.0;

        // 四元数朝向
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
        odom_.pose.pose.orientation = odom_quat;

        // 线速度和角速度
        odom_.twist.twist.linear.x = linear_velocity_;
        odom_.twist.twist.angular.z = angular_velocity_;

        odom_pub_.publish(odom_);//发布到话题/odom上


        // 发布 tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        tf_broadcaster_.sendTransform(odom_trans);
    }

    void KinematicsController::update(const ros::Time& time, const ros::Duration& period) {
        // 将计算得到的速度设置到关节
        left_front_wheel_joint_.setCommand(cmd_vel_[0]);
        right_front_wheel_joint_.setCommand(cmd_vel_[1]);
        left_back_wheel_joint_.setCommand(cmd_vel_[2]);
        right_back_wheel_joint_.setCommand(cmd_vel_[3]);

        computeBaseVelocityFromWheels();
        updateOdometry(time);
    }
    }

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::KinematicsController, controller_interface::ControllerBase)