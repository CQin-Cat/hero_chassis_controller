#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hero_chassis_controller/chassis_controller.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_controller_node");
    ros::NodeHandle nh;

    hardware_interface::EffortJointInterface eff_joint_interface;
    controller_manager::ControllerManager cm(nullptr, nh);

    hero_chassis_controller::EffortChassisController chassis_controller;

    if (!chassis_controller.init(&eff_joint_interface, nh, nh)) {
        ROS_ERROR("Failed to initialize chassis controller.");
        return -1;
    }

    ros::Rate rate(10);  //

    while (ros::ok()) {

        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - ros::Time::now();

        chassis_controller.update(current_time, period);

        cm.update(current_time, period);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


//
// Created by cqincat on 24-12-10.
//
