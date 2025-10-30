//
// Created by cqincat on 15/10/25.
//

#include <hardware_interface/joint_command_interface.h>
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {
double HeroChassisController::square(double x)
{
    return x * x;
}

void HeroChassisController::cb(const PIDConfig& config, uint32_t level)
{
    pid_front_left_.setGains(config.front_left_p, config.front_left_i, config.front_left_d, 0, 0);
    pid_front_right_.setGains(config.front_right_p, config.front_right_i, config.front_right_d, 0, 0);
    pid_back_left_.setGains(config.back_left_p, config.back_left_i, config.back_left_d, 0, 0);
    pid_back_right_.setGains(config.back_right_p, config.back_right_i, config.back_right_d, 0, 0);
    ROS_INFO("Update PID gains:");
    ROS_INFO("Front Left: P=%.2f, I=%.2f, D=%.2f", config.front_left_p, config.front_left_i, config.front_left_d);
    ROS_INFO("Front Right: P=%.2f, I=%.2f, D=%.2f", config.front_right_p, config.front_right_i, config.front_right_d);
    ROS_INFO("Back Left: P=%.2f, I=%.2f, D=%.2f", config.back_left_p, config.back_left_i, config.back_left_d);
    ROS_INFO("Back Right: P=%.2f, I=%.2f, D=%.2f", config.back_right_p, config.back_right_i, config.back_right_d);
}
//根据底盘参数(轮距/轴距/半径) 做逆运动学计算得到每个轮子的期望转速
void HeroChassisController::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!odomMode) {
        desired_front_left_velocity_ = msg->linear.x - msg->linear.y - (rx + ry) * msg->angular.z;
        desired_front_right_velocity_ = msg->linear.x + msg->linear.y + (rx + ry) * msg->angular.z;
        desired_back_left_velocity_ = msg->linear.x + msg->linear.y - (rx + ry) * msg->angular.z;
        desired_back_right_velocity_ = msg->linear.x - msg->linear.y + (rx + ry) * msg->angular.z;
    } else{
        geometry_msgs::Vector3Stamped world_vel;
        world_vel.header.stamp = ros::Time(0);
        world_vel.header.frame_id = "odom";
        world_vel.vector = msg->linear;

        geometry_msgs::Vector3Stamped base_vel;
        try {
            tf_listener_.waitForTransform("base_link","odom",ros::Time(0),ros::Duration(1,0));
            tf_listener_.transformVector("base_link", world_vel, base_vel); // 使用成员 tf listener
        } catch (tf::TransformException& ex) {
            ROS_WARN("tf error: %s", ex.what());
            return;
        }
        desired_front_left_velocity_  = base_vel.vector.x - base_vel.vector.y - (rx + ry) * msg->angular.z;
        desired_front_right_velocity_  = base_vel.vector.x + base_vel.vector.y + (rx + ry) * msg->angular.z;
        desired_back_left_velocity_  = base_vel.vector.x + base_vel.vector.y - (rx + ry) * msg->angular.z;
        desired_back_right_velocity_  = base_vel.vector.x - base_vel.vector.y + (rx + ry) * msg->angular.z;

    }
}
void HeroChassisController::computeWheelEfforts(const ros::Time& time, const ros::Duration& period)
{
    actual_front_left_velocity_ = front_left_joint_.getVelocity();
    actual_front_right_velocity_ = front_right_joint_.getVelocity();
    actual_back_left_velocity_ = back_left_joint_.getVelocity();
    actual_back_right_velocity_ = back_right_joint_.getVelocity();

    error_front_left_=desired_front_left_velocity_ - wheel_radius * actual_front_left_velocity_;
    error_front_right_=desired_front_right_velocity_ - wheel_radius * actual_front_right_velocity_;
    error_back_left_=desired_back_left_velocity_ - wheel_radius * actual_back_left_velocity_;
    error_back_right_=desired_back_right_velocity_ - wheel_radius * actual_back_right_velocity_;

    cmd_effort_front_left_ = pid_front_left_.computeCommand(error_front_left_, period);
    cmd_effort_front_right_ = pid_front_right_.computeCommand(error_front_right_, period);
    cmd_effort_back_left_ = pid_back_left_.computeCommand(error_back_left_, period);
    cmd_effort_back_right_ = pid_back_right_.computeCommand(error_back_right_, period);

    front_left_joint_.setCommand(cmd_effort_front_left_);
    front_right_joint_.setCommand(cmd_effort_front_right_);
    back_left_joint_.setCommand(cmd_effort_back_left_);
    back_right_joint_.setCommand(cmd_effort_back_right_);

    double a = 0., b = 0., c = 0.;
    a = square(cmd_effort_front_left_)+square(cmd_effort_front_right_)+square(cmd_effort_back_left_)+square(cmd_effort_back_right_);
    b = std::abs(cmd_effort_front_left_ * actual_front_left_velocity_)+std::abs(cmd_effort_front_right_ * actual_front_right_velocity_)+
        std::abs(cmd_effort_back_left_ * actual_back_left_velocity_)+std::abs(cmd_effort_back_right_ * actual_back_right_velocity_);
    c = square(actual_front_left_velocity_)+square(actual_front_right_velocity_)+
        square(actual_back_left_velocity_)+square(actual_back_right_velocity_);
    a *= effort_coeff_;
    c = c * velocity_coeff_ - power_offset_ - power_limit;
    // Root formula for quadratic equation in one variable
    double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
    for (auto joint : joint_handles_)
        joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
}

void HeroChassisController::odom_update(const ros::Time& time, const ros::Duration& period)
{
    // 各轮子实际的角速度
    actual_front_left_velocity_ = front_left_joint_.getVelocity();
    actual_front_right_velocity_ = front_right_joint_.getVelocity();
    actual_back_left_velocity_ = back_left_joint_.getVelocity();
    actual_back_right_velocity_ = back_right_joint_.getVelocity();

    // 四个轮子的合速度
    real_xvelocity =
        (actual_front_left_velocity_ + actual_front_right_velocity_ + actual_back_left_velocity_ + actual_back_right_velocity_) * wheel_radius / 4;
    real_yvelocity =
        (-actual_front_left_velocity_ + actual_front_right_velocity_ + actual_back_left_velocity_ - actual_back_right_velocity_) * wheel_radius / 4;
    real_thvelocity =
        (-actual_front_left_velocity_ + actual_front_right_velocity_ - actual_back_left_velocity_ + actual_back_right_velocity_) * wheel_radius / (4 * (rx + ry));

    // 正运动学计算
    dt = period.toSec();
    dx = (real_xvelocity * cos(th) - real_yvelocity * sin(th)) * dt;
    dy = (real_xvelocity * sin(th) + real_yvelocity * cos(th)) * dt;
    dth = real_thvelocity * dt;

    // 里程计累加
    x += dx;
    y += dy;
    th += dth;

    // 创建坐标转换
    geometry_msgs::TransformStamped odom_ts;
    //----设置头信息
    //odom_ts.header.seq = 100;
    odom_ts.header.stamp = time;
    odom_ts.header.frame_id = "odom";
    //----设置子级坐标系
    odom_ts.child_frame_id = "base_link";
    //----设置子级相对于父级的偏移量
    odom_ts.transform.translation.x = x;
    odom_ts.transform.translation.y = y;
    odom_ts.transform.translation.z = 0.0;
    //----设置四元数:将 欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,th);
    odom_ts.transform.rotation.x = qtn.getX();
    odom_ts.transform.rotation.y = qtn.getY();
    odom_ts.transform.rotation.z = qtn.getZ();
    odom_ts.transform.rotation.w = qtn.getW();

    odom_broadcaster.sendTransform(odom_ts);

    // 发布 Odometry 消息
    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = real_xvelocity;
    odom.twist.twist.linear.y = real_yvelocity;
    odom.twist.twist.angular.z = real_thvelocity;

    odom_pub.publish(odom);
}



bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

  joint_handles_.push_back(front_left_joint_);
  joint_handles_.push_back(front_right_joint_);
  joint_handles_.push_back(back_left_joint_);
  joint_handles_.push_back(back_right_joint_);

  controller_nh.getParam("wheel_track", wheel_track);
  controller_nh.getParam("wheel_base", wheel_base);
  controller_nh.getParam("wheel_radius", wheel_radius);
  controller_nh.getParam("odomMode", odomMode);
  controller_nh.getParam("power_limit", power_limit);
  if (!controller_nh.getParam("power/velocity_coeff", velocity_coeff_) ||
    !controller_nh.getParam("power/effort_coeff", effort_coeff_) ||
    !controller_nh.getParam("power/power_offset", power_offset_))
  {
      ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
      return false;
  }

  rx = wheel_track / 2.0;
  ry = wheel_base / 2.0;

  pid_front_left_.initPid(0, 0.0, 0.0, 0, 0);
  pid_front_right_.initPid(0, 0.0, 0.0, 0, 0);
  pid_back_left_.initPid(0, 0.0, 0.0, 0, 0);
  pid_back_right_.initPid(0, 0.0, 0.0, 0, 0);

  cmd_vel_sub = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmd_vel_cb, this);
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);

  server = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>>(controller_nh);
  cbType = boost::bind(&HeroChassisController::cb,this,_1,_2);
  server->setCallback(cbType);

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

    computeWheelEfforts(time, period);
    odom_update(time, period);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}
