#include<hero_chassis_controller/chassis_controller.h>


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

        joint_ = robot->getHandle(joint_name);

        return true;
    }//第一个初始化较为简单，只需要知道关节名字和一个简单的pid控制器

    bool ChassisPidController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        std::string joint_name;
        if (!n.getParam("joint", joint_name)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }//典型的初始化代码，先检测节点是否存在；“”joint“”具体指向应该在yaml文件里面写了，joint_name是为了简单记住名字，避免复制

        joint_ = robot->getHandle(joint_name);

        //n有订阅和发布的功能，这里通过在当前命名空间 n 下创建一个子命名空间 "pid"，并将其传递给 PID 控制器以加载对应的参数。
        if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
            return false;

        controller_state_publisher_.reset(
                new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                        (n, "state", 1));//该对象发布 control_msgs::JointControllerState 消息，该消息包含了 PID 增益、误差、命令等信息。可以订阅查看。

        //字面意思，就是一个订阅者。setCommandCB是一个CallBack函数，有更新目标值的作用
        sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &ChassisPidController::setCommandCB, this);

//        f_ = boost::bind(&ChassisPidController::dynamicReconfigCallback, this, _1, _2);
//        server_.setCallback(f_);

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

    void ChassisPidController::setCommand(double& cmd)
    {
        command_ = cmd;
    }

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

//    void ChassisPidController::dynamicReconfigCallback(hero_chassis_controller::ChassisPidConfig &config, uint32_t level) {
//        setGains(config.p, config.i, config.d, config.i_max, config.i_min, config.antiwindup);
//    }


    bool EffortChassisController::init(
            hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        front_left_joint_ =
                effort_joint_interface->getHandle("hero::left_front_wheel_joint");
        front_right_joint_ =
                effort_joint_interface->getHandle("hero::right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("hero::left_back_wheel_joint");
        back_right_joint_ =
                effort_joint_interface->getHandle("hero::right_back_wheel_joint");
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
//    MecanumWheelController::MecanumWheelController() : wheel_base_(0.5), track_width_(0.5), wheel_radius_(0.1) {}
//
//    MecanumWheelController::~MecanumWheelController() {}
//
//    bool MecanumWheelController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
//    {
//        // 获取四个麦克纳姆轮的关节句柄
//        left_front_wheel_ = hw->getHandle("left_front_wheel_joint");
//        right_front_wheel_ = hw->getHandle("right_front_wheel_joint");
//        left_back_wheel_ = hw->getHandle("left_back_wheel_joint");
//        right_back_wheel_ = hw->getHandle("right_back_wheel_joint");
//
//        // 获取参数
//        nh.param("wheel_base", wheel_base_, wheel_base_);
//        nh.param("track_width", track_width_, track_width_);
//        nh.param("wheel_radius", wheel_radius_, wheel_radius_);
//
//        // 订阅 "/cmd_vel" 话题
//        cmd_vel_sub_ = nh.subscribe("/cmd_vel", 1, &MecanumWheelController::cmdVelCallback, this);
//
//        return true;
//    }
//
//    void MecanumWheelController::update(const ros::Time& time, const ros::Durat我们直接从 `cmd_vel` 获取速度指令
//    }
//
//    void MecanumWheelController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
//    {
//        // 获取速度指令
//        double vx = msg->linear.x;  // 前进速度
//        double vy = msg->linear.y;  // 侧向速度
//        double omega = msg->angular.z;  // 旋转速度
//
//        // 计算四个轮子的期望转速
//        double v1 = (1 / wheel_radius_) * (vx - vy - (wheel_base_ + track_width_) * omega);
//        double v2 = (1 / wheel_radius_) * (vx + vy + (wheel_base_ + track_width_) * omega);
//        double v3 = (1 / wheel_radius_) * (vx + vy - (wheel_base_ + track_width_) * omega);
//        double v4 = (1 / wheel_radius_) * (vx - vy + (wheel_base_ + track_width_) * omega);
//
//        // 设置每个麦克纳姆轮的转速
//        left_front_wheel_.setCommand(v1);
//        right_front_wheel_.setCommand(v2);
//        left_back_wheel_.setCommand(v3);
//        right_back_wheel_.setCommand(v4);
//    }ion& period)
//    {
//        // 在这里你可以根据计算的期望转速来设置轮子速度
//        // 此示例不进行计算，假设

}
PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::EffortChassisController, controller_interface::ControllerBase)
//PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::MecanumWheelController, controller_interface::ControllerBase)


//
// Created by cqincat on 24-12-9.
//
