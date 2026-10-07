#include "drainy_controller/DrainyController.hpp"

namespace easynav
{
    DrainyController::DrainyController(){}

    DrainyController::~DrainyController(){}

    void DrainyController::on_initialize()
    {

        auto node = get_node();
        const std::string & plugin_name = this->get_plugin_name();

        node->declare_parameter(plugin_name + ".x_gain", x_gain_);
        node->declare_parameter(plugin_name + ".y_gain", y_gain_);
        node->declare_parameter(plugin_name + ".z_gain", z_gain_);
        node->declare_parameter(plugin_name + ".yaw_gain", yaw_gain_);
        node->declare_parameter(plugin_name + ".safety_radius", safety_radius_);

        node->get_parameter(plugin_name + ".x_gain", x_gain_);
        node->get_parameter(plugin_name + ".y_gain", y_gain_);
        node->get_parameter(plugin_name + ".z_gain", z_gain_);
        node->get_parameter(plugin_name + ".yaw_gain", yaw_gain_);
        node->get_parameter(plugin_name + ".safety_radius", safety_radius_);

        cloud_h_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node->get_name() + std::string("/") + plugin_name + "/cloud_detection/horizontal", 
            rclcpp::QoS(1).transient_local().reliable());

        cloud_v_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node->get_name() + std::string("/") + plugin_name + "/cloud_detection/vertical", 
            rclcpp::QoS(1).transient_local().reliable());

        RCLCPP_INFO(node->get_logger(), "%s plugin has been initialized", plugin_name.c_str());

    }

    void DrainyController::update_rt([[maybe_unused]] NavState & nav_state)
    {
        const auto & perceptions = nav_state.get_no_group<PointPerception>();

        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

        if (!nav_state.has("path") || !nav_state.has("robot_pose") || perceptions.empty()) {
            RCLCPP_DEBUG(get_node()->get_logger(), "No Path, No Points or No Robot Pose");
            return;
        }

        const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

        const auto & filtered = PointPerceptionsOpsView(perceptions)
            .filter({-detection_limit_, -detection_limit_, -0.1,},
                {detection_limit_, detection_limit_, 0.1,})
            .fuse(tf_info.map_frame)
            .collapse({NAN, NAN, robot_pose.pose.pose.position.z})
            .downsample(0.1)
            .as_points();

        Eigen::Vector4f min_xy, max_xy;
        pcl::getMinMax3D(filtered, min_xy, max_xy);
        pcl::toROSMsg(filtered, cloud_h_msg_);
        cloud_h_msg_.header.frame_id = tf_info.map_frame;
        cloud_h_msg_.header.stamp = get_node()->now();
        cloud_h_pub_->publish(cloud_h_msg_);

        const auto & z_filtered = PointPerceptionsOpsView(perceptions)
            .filter({detection_limit_ - 0.5, - 0.5, -5.0,},
                {detection_limit_ + 0.5, 0.5, 5.0,})
            .fuse(tf_info.map_frame)
            .downsample(0.1)
            .as_points();

        Eigen::Vector4f min_z, max_z;
        pcl::getMinMax3D(z_filtered, min_z, max_z);
        pcl::toROSMsg(z_filtered, cloud_v_msg_);
        cloud_v_msg_.header.frame_id = tf_info.map_frame;
        cloud_v_msg_.header.stamp = get_node()->now();
        cloud_v_pub_->publish(cloud_v_msg_);

        double height = min_z[2] + (max_z[2] - min_z[2]) / 2.0; 

        if(std::abs(height) < safety_vertical_){
            RCLCPP_WARN(get_node()->get_logger(), 
                "Imminent Collision due to Narrow Height. Safety limit has been set := %lf",
                height);
            height = safety_vertical_;
            
        }
        nav_state.set("height", height);

        // If navigation is IDLE, force zero velocity
        // con un goal constante no hay necesidad

        if (nav_state.has("navigation_state")) {
            const auto nav_state_val = nav_state.get<easynav::GoalManager::State>("navigation_state");
            if (nav_state_val == easynav::GoalManager::State::IDLE) {
                cmd_vel_.header.stamp = get_node()->now();
                cmd_vel_.twist.linear.x = 0.0;
                cmd_vel_.twist.linear.z = 0.0;
                cmd_vel_.twist.angular.z = 0.0;
                nav_state.set("cmd_vel", cmd_vel_);
                return;
            }
        }

        nav_msgs::msg::Path path = nav_state.get<nav_msgs::msg::Path>("path");
        if (path.poses.empty()) {
            // RCLCPP_INFO(get_node()->get_logger(), "Path is empty, Vel will be zero.");
            cmd_vel_.header.frame_id = path.header.frame_id;
            cmd_vel_.header.stamp = get_node()->now();
            cmd_vel_.twist.linear.x = 0.0;
            cmd_vel_.twist.linear.z = 0.0;
            cmd_vel_.twist.angular.z = 0.0;
            nav_state.set("cmd_vel", cmd_vel_);
            return;
        }

        // Limits

        double x_offset = 0.0;
        double y_offset = 0.0;
        double z_offset = 0.0;
        
        if(std::abs(min_z[2] - robot_pose.pose.pose.position.z) < safety_vertical_) {
            RCLCPP_WARN(get_node()->get_logger(), "Imminent Collision Detected in Z min level:= %lf", min_z[2]);
            z_offset = 0.5;
        }
        if(std::abs(max_z[2] - robot_pose.pose.pose.position.z) < safety_vertical_) {
            RCLCPP_WARN(get_node()->get_logger(), "Imminent Collision Detected in Z max level:= %lf", max_z[2]);
            z_offset = -0.5;
        }

        if(std::abs(min_xy[0]) < safety_radius_) {
            RCLCPP_WARN(get_node()->get_logger(), "Imminent Collision Detected in X := %lf", min_xy[0]);
            x_offset = -0.5;
        }
        if(0 < min_xy[1] && min_xy[1] < safety_radius_) {
            RCLCPP_WARN(get_node()->get_logger(), "Imminent Collision Detected in Y := %lf", min_xy[1]);
            x_offset = -0.5;
            y_offset = -0.5;
        }
        if(-safety_radius_ < min_xy[1] && min_xy[1] < 0.0) {
            RCLCPP_WARN(get_node()->get_logger(), "Imminent Collision Detected in Y := %lf", min_xy[1]);
            x_offset = -0.5;
            y_offset = 0.5;
        }

        const auto & goal_pose = path.poses.back().pose;

        // goal unreachabled

        tf2::Quaternion robot_q(
            robot_pose.pose.pose.orientation.x,
            robot_pose.pose.pose.orientation.y,
            robot_pose.pose.pose.orientation.z,
            robot_pose.pose.pose.orientation.w);
        double robot_yaw = tf2::getYaw(robot_q);

        tf2::Quaternion goal_q(
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w);
        double goal_yaw = tf2::getYaw(goal_q);

        double ex = goal_pose.position.x - robot_pose.pose.pose.position.x;
        double ey = goal_pose.position.y - robot_pose.pose.pose.position.y;
        double ez = goal_pose.position.z - robot_pose.pose.pose.position.z;
        
        if(std::isfinite(min_xy[1]) && std::isfinite(max_xy[1])){
            ey_medio_ = ((std::abs(max_xy[1]) - std::abs(min_xy[1])) / 2.0) - robot_pose.pose.pose.position.y; 
        }

        double e_angle = std::atan2(ey, ex);   

        double distance = std::hypot(ex, ey);

        // Always use the shortest rotation direction.
        double eyaw = std::atan2(std::sin(e_angle - robot_yaw),
                      std::cos(e_angle - robot_yaw));

        
        if( distance < convergence_limit_) {
            RCLCPP_INFO(get_node()->get_logger(), "Goal is near at %lf", distance);
            eyaw = goal_yaw - robot_yaw;
            x_offset = -0.1*distance;
            y_offset = -0.5*ey_medio_;
        }

        if (std::abs(eyaw) > yaw_limit_ ) {
            RCLCPP_INFO(get_node()->get_logger(), "Angle error too big:= %lf", eyaw);
            cmd_vel_.header.stamp = get_node()->now();
            cmd_vel_.twist.linear.x = 0.0;
            cmd_vel_.twist.linear.y = 0.0;
            cmd_vel_.twist.linear.z = 0.0;
            const double angular_velocity = eyaw * yaw_gain_;
            cmd_vel_.twist.angular.z = std::clamp(angular_velocity,
                                                  -vel_angular_max_,
                                                  vel_angular_max_);
            nav_state.set("cmd_vel", cmd_vel_);
            return;
        }    

        // The robot only advances after its heading is within yaw_limit_.
        // Drive straight toward the point in the robot X direction.
        cmd_vel_.header.frame_id = path.header.frame_id;
        cmd_vel_.header.stamp = get_node()->now();
        cmd_vel_.twist.linear.x = std::clamp(distance * x_gain_ + x_offset,
                                             -vel_lineal_max_,
                                             vel_lineal_max_);
        cmd_vel_.twist.linear.y = std::clamp(ey_medio_ * y_gain_ + y_offset,
                                             -vel_lineal_max_,
                                             vel_lineal_max_);
        cmd_vel_.twist.linear.z = std::clamp(ez * z_gain_ + z_offset,
                                        -vel_lineal_max_,
                                        vel_lineal_max_);
        const double angular_velocity = eyaw * yaw_gain_;
        cmd_vel_.twist.angular.z = std::clamp(angular_velocity,
                                                -vel_angular_max_,
                                                vel_angular_max_);
        nav_state.set("cmd_vel", cmd_vel_);
    }

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyController, easynav::ControllerMethodBase)