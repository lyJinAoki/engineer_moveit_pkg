#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <thread>
#include <atomic>


class MultiRobotPlanner : public rclcpp::Node
{
public:
    MultiRobotPlanner() : Node("multi_robot_planner")
    {
        RCLCPP_INFO(this->get_logger(), "MultiRobotPlanner node created.");
        
        // 创建一个单独的线程来运行事件循环
        event_loop_thread_ = std::thread([this]() {
            while (is_running_ && rclcpp::ok()) {
                rclcpp::spin_some(this->shared_from_this());
            }
        });
    }

    ~MultiRobotPlanner()
    {
        // 停止事件循环线程
        is_running_ = false;
        if (event_loop_thread_.joinable()) {
            event_loop_thread_.join();
        }
    }
    
    void initializeMoveGroups()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterfaces.");
        try {
            // Initialize MoveGroupInterfaces for arm, chassis, and station
            arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
            whole_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "whole");
            station_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "station");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterfaces initialized successfully.");

            // Set parameters for arm_group_
            arm_group_->setPlanningTime(2.0); // 设置规划时间
            arm_group_->setNumPlanningAttempts(50); // 设置规划尝试次数
            arm_group_->setGoalJointTolerance(5.0e-2); // 设置关节目标容差
            arm_group_->setGoalPositionTolerance(1.0e-2); // 设置位置目标容差
            arm_group_->setGoalOrientationTolerance(1.0e-2); // 设置姿态目标容差

            // Set parameters for whole_group_
            whole_group_->setPlanningTime(10.0); // 设置规划时间
            whole_group_->setNumPlanningAttempts(50); // 设置规划尝试次数
            whole_group_->setGoalJointTolerance(0.1); // 设置关节目标容差
            whole_group_->setGoalPositionTolerance(0.1); // 设置位置目标容差
            whole_group_->setGoalOrientationTolerance(0.1); // 设置姿态目标容差

            // Set parameters for station_group_
            station_group_->setPlanningTime(10.0); // 设置规划时间
            station_group_->setNumPlanningAttempts(20); // 设置规划尝试次数
            station_group_->setGoalJointTolerance(1.0e-4); // 设置关节目标容差
            station_group_->setGoalPositionTolerance(1.0e-3); // 设置位置目标容差
            station_group_->setGoalOrientationTolerance(1.0e-3); // 设置姿态目标容差


        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterfaces: %s", e.what());
        }
    }

    geometry_msgs::msg::PoseStamped getUserInput(const std::string &group_name)
    {
        RCLCPP_INFO(this->get_logger(), "请输入 %s 的目标位姿（x y z yaw pitch roll）：", group_name.c_str());
        double x, y, z, yaw, pitch, roll;
        std::cin >> x >> y >> z >> yaw >> pitch >> roll;
        // Convert RPY to quaternion
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = q.x();
        quaternion.y = q.y();
        quaternion.z = q.z();
        quaternion.w = q.w();

        // Create the target pose
        geometry_msgs::msg::PoseStamped target_pose;
        if (group_name == "station"){
            target_pose.header.frame_id = "J1";
        } else if (group_name == "arm"){
            target_pose.header.frame_id = "J1";
        }
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation = quaternion;

        return target_pose;
    }


    void printTrajectoryPath(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        RCLCPP_INFO(this->get_logger(), "轨迹路径：");
        for (size_t i = 0; i < plan.trajectory.joint_trajectory.points.size(); ++i) {
            const auto &point = plan.trajectory.joint_trajectory.points[i];
            std::stringstream ss;
            ss << "点 " << i << ": ";
            for (size_t j = 0; j < point.positions.size(); ++j) {
                ss << "关节 " << j << ": " << point.positions[j] << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
    }

    bool planStationMotion(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        RCLCPP_INFO(this->get_logger(), "规划 station 路径...");
        if (!station_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface for station is not initialized.");
            return false;
        }
        station_group_->setPoseTarget(target_pose.pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (station_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "station 路径规划成功！");
            station_group_->execute(plan);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "station 路径规划失败");
            return false;
        }
    }

    geometry_msgs::msg::PoseStamped calculatePushInPose(const geometry_msgs::msg::PoseStamped &pre_pushin_pose, double offset)
    {
        // 创建 push_in 目标位姿
        geometry_msgs::msg::PoseStamped push_in_pose = pre_pushin_pose;

        // 提取 pre_pushin 位姿的旋转信息
        tf2::Quaternion q(
            pre_pushin_pose.pose.orientation.x,
            pre_pushin_pose.pose.orientation.y,
            pre_pushin_pose.pose.orientation.z,
            pre_pushin_pose.pose.orientation.w
        );

        // 计算末端执行器的 x 轴方向向量
        tf2::Vector3 x_axis(1, 0, 0); // 局部坐标系中的 x 轴
        tf2::Vector3 x_axis_global = tf2::quatRotate(q, x_axis); // 转换到全局坐标系

        // 沿 x 轴方向移动 offset 距离
        push_in_pose.pose.position.x += x_axis_global.x() * offset;
        push_in_pose.pose.position.y += x_axis_global.y() * offset;
        push_in_pose.pose.position.z += x_axis_global.z() * offset;

        return push_in_pose;
    }

    bool planArmMotion(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        RCLCPP_INFO(this->get_logger(), "规划机械臂路径...");
        if (!arm_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface for arm is not initialized.");
            return false;
        }
        arm_group_->setPoseTarget(target_pose.pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "机械臂pre_pushin路径规划成功！");
            // Update the robot state to reflect the new station position
            arm_group_->execute(plan);
            printTrajectoryPath(plan);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "机械臂路径规划失败");
            return false;
        }
    }

    bool executePushInMotion(const geometry_msgs::msg::PoseStamped &pre_pushin_pose)
    {

        // 计算 push_in 目标位姿：沿末端执行器的 x 轴方向移动 20 cm
        double offset = -0.13; // 13 cm
        geometry_msgs::msg::PoseStamped push_in_pose = calculatePushInPose(pre_pushin_pose, offset);

        // 依次规划并执行 pre_pushin 和 push_in 动作
        bool pre_pushin_success = planArmMotion(pre_pushin_pose);
        if (pre_pushin_success) {
            RCLCPP_INFO(this->get_logger(), "pre_pushin 动作执行成功！");
            bool push_in_success = planArmMotion(push_in_pose);
            if (push_in_success) {
                RCLCPP_INFO(this->get_logger(), "push_in 动作执行成功！");
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "push_in 动作执行失败");
                return false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "pre_pushin 动作执行失败");
            return false;
        }
    }

    bool moveBaseAndReplan(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        RCLCPP_WARN(this->get_logger(), "机械臂路径规划失败，尝试移动底盘...");
        if (!whole_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface for chassis is not initialized.");
            return false;
        }

        // 设置底盘的目标位置
        whole_group_->setPoseTarget(target_pose.pose);
        moveit::planning_interface::MoveGroupInterface::Plan base_plan;
        bool success = (whole_group_->plan(base_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "底盘移动成功！");

            // // 获取当前关节状态
            // auto current_state = whole_group_->getCurrentState();
            // if (!current_state) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to get current state!");
            //     return false;
            // }

            // // 打印当前关节状态
            // RCLCPP_INFO(this->get_logger(), "我成功getCurrentState了，yeah");
            // std::vector<std::string> joint_names = whole_group_->getJointNames();
            // std::vector<double> joint_values;
            // current_state->copyJointGroupPositions("whole", joint_values);
            
            // // 过滤轨迹点
            // std::vector<std::string> chassis_joints = {"chassis_x", "chassis_y", "chassis_theta"};
            // std::vector<trajectory_msgs::msg::JointTrajectoryPoint> new_trajectory_points;
            // for (const auto& point : base_plan.trajectory.joint_trajectory.points) {
            //     trajectory_msgs::msg::JointTrajectoryPoint new_point;
            //     for (const auto& joint : joint_names) {
            //         auto index = std::distance(base_plan.trajectory.joint_trajectory.joint_names.begin(),
            //                                 std::find(base_plan.trajectory.joint_trajectory.joint_names.begin(),
            //                                             base_plan.trajectory.joint_trajectory.joint_names.end(), joint));
            //         if (std::find(chassis_joints.begin(), chassis_joints.end(), joint) != chassis_joints.end()) {
            //             // 如果是底盘关节，保留速度和加速度
            //             new_point.positions.push_back(point.positions[index]);
            //             if (!point.velocities.empty()) {
            //                 new_point.velocities.push_back(point.velocities[index]);
            //             } else {
            //                 new_point.velocities.push_back(0.0); // 如果原始轨迹没有速度信息，设置为0
            //             }
            //             if (!point.accelerations.empty()) {
            //                 new_point.accelerations.push_back(point.accelerations[index]);
            //             } else {
            //                 new_point.accelerations.push_back(0.0); // 如果原始轨迹没有加速度信息，设置为0
            //             }
            //         } else {
            //             // 如果不是底盘关节，使用当前关节状态
            //             new_point.positions.push_back(current_state->getVariablePosition(joint));
            //             new_point.velocities.push_back(0.0); // 设置速度为0
            //             new_point.accelerations.push_back(0.0); // 设置加速度为0
            //         }
            //     }
            //     new_trajectory_points.push_back(new_point);
            // }

            // // 更新 base_plan 轨迹
            // base_plan.trajectory.joint_trajectory.joint_names = joint_names;
            // base_plan.trajectory.joint_trajectory.points = new_trajectory_points;  
            
            RCLCPP_INFO(this->get_logger(), "Joint Names:");
            for (const auto& joint : base_plan.trajectory.joint_trajectory.joint_names) {
                RCLCPP_INFO(this->get_logger(), "- %s", joint.c_str());
            }

            RCLCPP_INFO(this->get_logger(), "Trajectory Points:");
            for (size_t i = 0; i < base_plan.trajectory.joint_trajectory.points.size(); i++) {
                const auto& point = base_plan.trajectory.joint_trajectory.points[i];
                RCLCPP_INFO(this->get_logger(), "Point %zu:", i);
                RCLCPP_INFO(this->get_logger(), "  Positions:");
                for (size_t j = 0; j < point.positions.size(); j++) {
                    RCLCPP_INFO(this->get_logger(), "    %s: %f", base_plan.trajectory.joint_trajectory.joint_names[j].c_str(), point.positions[j]);
                }
                if (!point.velocities.empty()) {
                    RCLCPP_INFO(this->get_logger(), "  Velocities:");
                    for (size_t j = 0; j < point.velocities.size(); j++) {
                        RCLCPP_INFO(this->get_logger(), "    %s: %f", base_plan.trajectory.joint_trajectory.joint_names[j].c_str(), point.velocities[j]);
                    }
                }
                if (!point.accelerations.empty()) {
                    RCLCPP_INFO(this->get_logger(), "  Accelerations:");
                    for (size_t j = 0; j < point.accelerations.size(); j++) {
                        RCLCPP_INFO(this->get_logger(), "    %s: %f", base_plan.trajectory.joint_trajectory.joint_names[j].c_str(), point.accelerations[j]);
                    }
                }
                double time_from_start = static_cast<double>(point.time_from_start.sec) +
                                        static_cast<double>(point.time_from_start.nanosec) / 1e9;
                RCLCPP_INFO(this->get_logger(), "  Time from start: %f", time_from_start);
            }
            // 执行修改后的轨迹
            RCLCPP_INFO(this->get_logger(), "Executing modified trajectory...");
            whole_group_->execute(base_plan);
            return planArmMotion(target_pose);
        } else {
            RCLCPP_ERROR(this->get_logger(), "底盘移动失败！");
            return false;
        }
    }


private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> whole_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> station_group_;
    std::thread event_loop_thread_;
    std::atomic<bool> is_running_{true};
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiRobotPlanner>();

    // Initialize MoveGroupInterfaces after the node is fully constructed
    node->initializeMoveGroups();

    // Step 1: Get station target pose from user input
    auto station_target_pose = node->getUserInput("station");

    // Step 2: Plan station motion
    if (node->planStationMotion(station_target_pose)) {

        // Step 3: Get arm target pose from user input
        auto arm_target_pose = node->getUserInput("arm");

        // Step 4: Execute push-in motion (pre_pushin + push_in)
        if (!node->executePushInMotion(arm_target_pose)) {
            RCLCPP_ERROR(node->get_logger(), "机械臂 push-in 动作失败，开始移动底盘");

            if (!node->moveBaseAndReplan(arm_target_pose)) {
                RCLCPP_ERROR(node->get_logger(), "底盘移动失败，任务终止。");
            }
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "兑换站路径规划失败，任务终止。");
    }

    rclcpp::shutdown();
    return 0;
}