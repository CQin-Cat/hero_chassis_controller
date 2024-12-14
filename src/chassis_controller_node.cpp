#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hero_chassis_controller/chassis_controller.h>
#include <control_msgs/JointControllerState.h>

class ChassisControllerNode {
public:
    ChassisControllerNode()
        : nh_(), rate_(10), eff_joint_interface_(), cm_(nullptr, nh_) {

        state_sub_ = nh_.subscribe("/state", 10, &ChassisControllerNode::stateCallback, this);

        if (!chassis_controller_.init(&eff_joint_interface_, nh_, nh_)) {
            ROS_ERROR("Failed to initialize chassis controller.");
            ros::shutdown();
        }
    }

    void spin() {
        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            ros::Duration period = current_time - last_time_;

            chassis_controller_.update(current_time, period);

            cm_.update(current_time, period);//该方法会检查所有已加载的控制器，并调用它们的 update 方法。

            ros::spinOnce();
            rate_.sleep();

            last_time_ = current_time;
        }
    }

private:
    void stateCallback(const control_msgs::JointControllerState::ConstPtr& msg) {
        ROS_INFO("Joint: %s", msg->header.frame_id.c_str());
        ROS_INFO("Setpoint: %f, Process Value: %f, Error: %f",
                 msg->set_point,
                 msg->process_value,
                 msg->error);
    }

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Rate rate_;
    ros::Time last_time_;

    hardware_interface::EffortJointInterface eff_joint_interface_;
    controller_manager::ControllerManager cm_;
    hero_chassis_controller::EffortChassisController chassis_controller_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_controller_node");

    ChassisControllerNode controller_node;

    controller_node.spin();

    return 0;
}

//
// Created by cqincat on 24-12-10.
//
