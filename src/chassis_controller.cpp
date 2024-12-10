#include<hero_chassis_controller/chassis_controller.h>
#include<pluginlib/class_list_macros.h>

namespace hero_chassis_controller {

    ChassisPidController::ChassisPidController()
            : command_(0), loop_count_(0)//初始化列表使函数更加直观清晰
    {}

    ChassisPidController::~ChassisPidController()
    {
        sub_command_.shutdown();//关闭订阅者
    }

    bool ChassisPidController::init(hardware_interface::EffortJointInterface *robot,
                                       const std::string &joint_name, const control_toolbox::Pid &pid)
    {
        pid_controller_ = pid;

        // Get joint handle from hardware interface
        joint_ = robot->getHandle(joint_name);

        return true;
    }//第一个初始化较为简单，只需要知道关节名字和一个简单的pid控制器

    bool ChassisPidController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        // Get joint name from parameter server
        std::string joint_name;
        if (!n.getParam("joint", joint_name)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }//典型的初始化代码，先检测节点是否存在；“”joint“”具体指向应该在yaml文件里面写了，joint_name是为了简单记住名字，避免复制

        // Get joint handle from hardware interface
        joint_ = robot->getHandle(joint_name);

        // Load PID Controller using gains set on parameter server，n有订阅和发布的功能，这里通过在当前命名空间 n 下创建一个子命名空间 "pid"，并将其传递给 PID 控制器以加载对应的参数。
        if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
            return false;

        // Start realtime state publisher，n提供了接口给发布者从这里可以看出相比上一个初始化函数，这里可以实现rqt-reconfigure的功能
        controller_state_publisher_.reset(
                new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                        (n, "state", 1));//该对象发布 control_msgs::JointControllerState 消息，该消息包含了 PID 增益、误差、命令等信息。

        // Start command subscriber，字面意思，就是一个订阅者。setCommandCB是一个CallBack函数，有更新目标值的作用
        sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &ChassisPidController::setCommandCB, this);

        return true;
    }

    void ChassisPidController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void ChassisPidController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void ChassisPidController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);//dummy,手动选择要不要积分限幅
    }

    void ChassisPidController::printDebug()
    {
        pid_controller_.printValues();
    }

    std::string ChassisPidController::getJointName()
    {
        return joint_.getName();
    }

// Set the joint velocity command
    void ChassisPidController::setCommand()
    {
        command_ = cmd;
    }

// Return the current velocity command
    void ChassisPidController::getCommand(double& cmd)
    {
        cmd = command_;
    }

    void ChassisPidController::starting(const ros::Time& time)
    {
        command_ = 0.0;
        pid_controller_.reset();
    }

    void ChassisPidController::update(const ros::Time& time, const ros::Duration& period)
    {
        double error = command_ - joint_.getVelocity();

        // Set the PID error and compute the PID command with nonuniform time
        // step size. The derivative error is computed from the change in the error
        // and the timestep dt.
        double commanded_effort = pid_controller_.computeCommand(error, period);

        joint_.setCommand(commanded_effort);

        if(loop_count_ % 10 == 0)
        {
            if(controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = command_;
                controller_state_publisher_->msg_.process_value = joint_.getVelocity();
                controller_state_publisher_->msg_.error = error;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = commanded_effort;//

                double dummy;
                bool antiwindup;
                getGains(controller_state_publisher_->msg_.p,
                         controller_state_publisher_->msg_.i,
                         controller_state_publisher_->msg_.d,
                         controller_state_publisher_->msg_.i_clamp,
                         dummy,
                         antiwindup);//getGains() 函数从 PID 控制器中提取当前的增益值，并将它们填充到 controller_state_publisher_->msg_ 中
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();//在这行代码中，消息发布器 controller_state_publisher_ 会将填充了 PID 增益、误差等信息的消息实时发布出去。
            }
        }
        loop_count_++;
    }

    void ChassisPidController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
    {
        command_ = msg->data;
    }

    bool EffortChassisController::init(
            hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        front_left_joint_ =
                effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ =
                effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ =
                effort_joint_interface->getHandle("right_back_wheel_joint");
        if (!front_left_pid_controller_.init(effort_joint_interface, controller_nh)) {
            ROS_ERROR("Failed to initialize front left wheel Pid Controller.");
            return false;
        }
        if (!front_right_pid_controller_.init(effort_joint_interface, controller_nh)) {
            ROS_ERROR("Failed to initialize front right wheel Pid Controller.");
            return false;
        }
        if (!back_left_pid_controller_.init(effort_joint_interface, controller_nh)) {
            ROS_ERROR("Failed to initialize back left wheel Pid Controller.");
            return false;
        }
        if (!back_right_pid_controller_.init(effort_joint_interface, controller_nh)) {
            ROS_ERROR("Failed to initialize back right wheel Pid Controller.");
            return false;
        }

        return true;
    }

    void EffortChassisController::update(const ros::Time &time, const ros::Duration &period) {

        double cmd_left_front, cmd_right_front, cmd_left_back, cmd_right_back;

        front_left_pid_controller_.update(time, period);
        front_left_pid_controller_.getCommand(cmd_left_front);

        front_right_pid_controller_.update(time, period);
        front_right_pid_controller_.getCommand(cmd_right_front);

        back_left_pid_controller_.update(time, period);
        back_left_pid_controller_.getCommand(cmd_left_back);

        back_right_pid_controller_.update(time, period);
        back_right_pid_controller_.getCommand(cmd_right_back);

        front_left_joint_.setCommand(cmd_left_front);
        front_right_joint_.setCommand(cmd_right_front);
        back_left_joint_.setCommand(cmd_left_back);
        back_right_joint_.setCommand(cmd_right_back);
    }

} // namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::ChassisPidController, controller_interface::ControllerBase)


//
// Created by cqincat on 24-12-9.
//
