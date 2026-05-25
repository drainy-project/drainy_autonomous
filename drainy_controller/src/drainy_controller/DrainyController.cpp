#include "drainy_controller/DrainyController.hpp"


namespace easynav
{
    DrainyController::DrainyController(){}

    DrainyController::~DrainyController(){}

    void DrainyController::on_initialize()
    {

        auto node = get_node();
        const std::string & plugin_name = this->get_plugin_name();

        RCLCPP_INFO(node->get_logger(), "%s plugin has been initialized", plugin_name.c_str());


    }

    void DrainyController::update_rt([[maybe_unused]] NavState & nav_state)
    {
        // If navigation is IDLE, force zero velocity
        if (nav_state.has("navigation_state")) {
            const auto nav_state_val = nav_state.get<easynav::GoalManager::State>("navigation_state");
            if (nav_state_val == easynav::GoalManager::State::IDLE) {
            cmd_vel_.header.stamp = get_node()->now();
            cmd_vel_.twist.linear.x = 0.0;
            cmd_vel_.twist.angular.z = 0.0;
            nav_state.set("cmd_vel", cmd_vel_);
            return;
            }
        }

        if (!nav_state.has("path") || !nav_state.has("robot_pose") || !nav_state.has("points")) {
            RCLCPP_INFO(get_node()->get_logger(), "No Path, No Points or No Robot Pose");
            return;
        }

        nav_msgs::msg::Path path = nav_state.get<nav_msgs::msg::Path>("path");
        if (path.poses.empty()) {
            RCLCPP_INFO(get_node()->get_logger(), "Path is empty, Vel will be zero.");
            cmd_vel_.header.frame_id = path.header.frame_id;
            cmd_vel_.header.stamp = get_node()->now();
            cmd_vel_.twist.linear.x = 0.0;
            cmd_vel_.twist.linear.y = 0.0;
            cmd_vel_.twist.linear.z = 0.0;
            cmd_vel_.twist.angular.z = 0.0;
            nav_state.set("cmd_vel", cmd_vel_);
            return;
        }

        const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

        const auto & goal_pose = path.poses.back().pose;

        double robot_roll, robot_pitch, robot_yaw;
        double goal_roll, goal_pitch, goal_yaw;
        tf2::Quaternion robot_q(
            robot_pose.pose.pose.orientation.x,
            robot_pose.pose.pose.orientation.y,
            robot_pose.pose.pose.orientation.z,
            robot_pose.pose.pose.orientation.w);
        tf2::Matrix3x3 robot_m(robot_q);
        robot_m.getRPY(robot_roll, robot_pitch, robot_yaw);

        tf2::Quaternion goal_q(
            robot_pose.pose.pose.orientation.x,
            robot_pose.pose.pose.orientation.y,
            robot_pose.pose.pose.orientation.z,
            robot_pose.pose.pose.orientation.w);
        tf2::Matrix3x3 goal_m(robot_q);
        goal_m.getRPY(goal_roll, goal_pitch, goal_yaw);

        double ex = goal_pose.position.x - robot_pose.pose.pose.position.x;
        double ey = goal_pose.position.y - robot_pose.pose.pose.position.y;
        double ez = goal_pose.position.z - robot_pose.pose.pose.position.z;
        double eyaw = goal_yaw - robot_yaw;

        double x_gain = 0.2;
        double y_gain = 0.5;
        double z_gain = 0.2;
        double yaw_gain = 0.5;

        double vel_lineal_max = 1.0;
        double vel_angular_max = 1.0;

        // RCLCPP_INFO(get_node()->get_logger(), "X vel := %f", ex*x_gain);

        cmd_vel_.header.frame_id = path.header.frame_id;
        cmd_vel_.header.stamp = get_node()->now();
        cmd_vel_.twist.linear.x = std::abs(ex*x_gain) > vel_lineal_max ? vel_lineal_max * std::abs(ex*x_gain)/ex*x_gain : ex*x_gain;
        cmd_vel_.twist.linear.y = std::abs(ey*y_gain) > vel_lineal_max ? vel_lineal_max * std::abs(ey*y_gain)/ey*y_gain : ey*y_gain;
        cmd_vel_.twist.linear.z = std::abs(ez*z_gain) > vel_lineal_max ? vel_lineal_max * std::abs(ez*z_gain)/ez*z_gain : ez*z_gain;
        cmd_vel_.twist.angular.z = std::abs(eyaw*yaw_gain) > vel_angular_max ? vel_angular_max * std::abs(eyaw*yaw_gain)/eyaw*yaw_gain : eyaw*yaw_gain;

        nav_state.set("cmd_vel", cmd_vel_);
        
    }

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyController, easynav::ControllerMethodBase)