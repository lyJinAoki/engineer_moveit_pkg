#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.hpp>


class MultiRobotPlanner : public rclcpp::Node
{
public:
    MultiRobotPlanner() : Node("multi_robot_planner")
    {
        RCLCPP_INFO(this->get_logger(), "MultiRobotPlanner node created.");
    }

    void initializeMoveGroups()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterfaces.");
        try {
            // Initialize MoveGroupInterfaces for arm, chassis, and station
            arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
            chassis_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "chassis");
            station_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "station");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterfaces initialized successfully.");

            // Set parameters for arm_group_
            arm_group_->setPlanningTime(2.0); // 设置规划时间
            arm_group_->setNumPlanningAttempts(50); // 设置规划尝试次数
            arm_group_->setGoalJointTolerance(5.0e-2); // 设置关节目标容差
            arm_group_->setGoalPositionTolerance(5.0e-2); // 设置位置目标容差
            arm_group_->setGoalOrientationTolerance(5.0e-2); // 设置姿态目标容差

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
            target_pose.header.frame_id = "root_link";
        } else if (group_name == "arm"){
            target_pose.header.frame_id = "root_link";
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

    bool executePushInMotion(const std::string &group_name)
    {
        // 获取用户输入的 pre_pushin 目标位姿
        geometry_msgs::msg::PoseStamped pre_pushin_pose = getUserInput(group_name);

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
    // bool moveBaseAndReplan(const geometry_msgs::msg::PoseStamped &target_pose)
    // {
    //     RCLCPP_WARN(this->get_logger(), "机械臂路径规划失败，尝试移动底盘...");
    //     if (!chassis_group_) {
    //         RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface for chassis is not initialized.");
    //         return false;
    //     }

    //     chassis_group_->setRandomTarget();
    //     moveit::planning_interface::MoveGroupInterface::Plan base_plan;

    //     bool success = (chassis_group_->plan(base_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //     if (success) {
    //         RCLCPP_INFO(this->get_logger(), "底盘移动成功！");
    //         chassis_group_->execute(base_plan);
    //         return planArmMotion(target_pose);
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "底盘移动失败！");
    //         return false;
    //     }
    // }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> chassis_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> station_group_;
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

        // Step 4: Execute push-in motion (pre_pushin + push_in)
        if (!node->executePushInMotion("arm")) {
            RCLCPP_ERROR(node->get_logger(), "机械臂 push-in 动作失败，开始移动底盘");
            // if (!node->moveBaseAndReplan(arm_target_pose)) {
            //     RCLCPP_ERROR(node->get_logger(), "底盘移动失败，任务终止。");
            // }
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "兑换站路径规划失败，任务终止。");
    }

    rclcpp::shutdown();
    return 0;
}