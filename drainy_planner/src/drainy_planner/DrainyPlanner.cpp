#include "drainy_planner/DrainyPlanner.hpp"


namespace easynav
{
    DrainyPlanner::DrainyPlanner(){}

    DrainyPlanner::~DrainyPlanner(){}

    void DrainyPlanner::on_initialize()
    {

        auto node = get_node();
        const std::string & plugin_name = this->get_plugin_name();

        node->declare_parameter<double>(plugin_name + ".min_error", 0.1);

        node->get_parameter<double>(plugin_name + ".min_error", min_error_);

        RCLCPP_INFO(node->get_logger(), "%s plugin has been initialized", plugin_name.c_str());

        path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);

        detection_pub_ =node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/detection", 10);
    }

    void DrainyPlanner::update([[maybe_unused]] NavState & nav_state)
    {
        if (!nav_state.has("goals") || !nav_state.has("robot_pose")) {
            return;
        }

        const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
        if (goals.goals.empty()) {
            nav_state.set("path", current_path_);
            return;
        }

        const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
        const auto & path = nav_state.get<nav_msgs::msg::Path>("path");
        const auto & perceptions = nav_state.get<PointPerceptions>("points");

        if (!current_path_.poses.empty()) {
            double dx = current_path_.poses.front().pose.position.x - robot_pose.pose.pose.position.x;
            double dy = current_path_.poses.front().pose.position.y - robot_pose.pose.pose.position.y;
            double dz = current_path_.poses.front().pose.position.z - robot_pose.pose.pose.position.z;
            error_ = std::hypot(dx, dy, dz);
        }

        if (!current_path_.poses.empty()) {
            double dx = current_path_.poses.back().pose.position.x - robot_pose.pose.pose.position.x;
            double dy = current_path_.poses.back().pose.position.y - robot_pose.pose.pose.position.y;
            double dz = current_path_.poses.back().pose.position.z - robot_pose.pose.pose.position.z;
            local_error_ = std::hypot(dx, dy, dz);
        }

        RCLCPP_INFO(get_node()->get_logger(), "local:=  %f error:=  %f", local_error_, error_);
        
        if(error_ < min_error_){error_updated_ = true;}
        
        if(error_updated_){
            RCLCPP_INFO(get_node()->get_logger(), "Path updated");
            geometry_msgs::msg::Pose goal;
            goal.position.x = robot_pose.pose.pose.position.x + 7.0;
            goal.position.y = robot_pose.pose.pose.position.y;
            goal.position.z = robot_pose.pose.pose.position.z;
            auto poses = get_poses(perceptions, robot_pose.pose.pose, goal);
            RCLCPP_INFO(get_node()->get_logger(), "Poses lenght:= %ld", current_path_.poses.size());
            RCLCPP_INFO(get_node()->get_logger(), "Set poses");
            current_path_.poses.clear();
            current_path_.header.stamp = get_node()->now();
            current_path_.header.frame_id = goals.header.frame_id;
            for (const auto & pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = goals.header.frame_id;
            pose_stamped.header.stamp = current_path_.header.stamp;
            pose_stamped.pose = pose;
            current_path_.poses.push_back(pose_stamped);
            }
        }

        if (local_error_ < min_error_ && !error_updated_) {
            RCLCPP_INFO(get_node()->get_logger(), "Path reached");
            current_path_.poses.erase(current_path_.poses.end());
        }
        error_updated_ = false;
        nav_state.set("path", current_path_);
        path_pub_->publish(current_path_); 
        
    }

    std::vector<geometry_msgs::msg::Pose> DrainyPlanner::get_poses(
        const std::vector<std::shared_ptr<easynav::PointPerception>>& perceptions,
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & goal)
    {
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        std::vector<geometry_msgs::msg::Pose> path;
        double x_acum = 0, y_acum = 0, z_acum = 0;  
        size_t real_points = 0;
        double forward_increment = 2.0;
        double min_distance = 3.0;
        double window_distance = 0.2; 
        int max_windows = 3;

        //path.push_back(start);

        for (unsigned i=0; i<max_windows; i++)
        {
            const auto & filtered = PointPerceptionsOpsView(perceptions)
            .filter({(min_distance), -5.0, -5.0}, {(min_distance + window_distance), 5.0, 5.0})
            .fuse(tf_info.robot_frame)
            // .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
            // .collapse({0.1, NAN, NAN})
            .downsample(0.1)
            .as_points();

            for (const auto & point : filtered) {
                if(!std::isnan(point.x) || !std::isnan(point.y) || !std::isnan(point.z)) {
                    x_acum += (point.x + start.position.x);
                    y_acum += (point.y + start.position.y);
                    z_acum += (point.z + start.position.z);
                    real_points++;
                }
            }

            geometry_msgs::msg::Pose avg_pose;
            avg_pose.position.x = (real_points > 0) ? (x_acum / real_points): 0.0;
            avg_pose.position.y = (real_points > 0) ? (y_acum / real_points): 0.0;
            avg_pose.position.z = (real_points > 0) ? (z_acum / real_points): 0.0;

            // RCLCPP_INFO(get_node()->get_logger(), "x:=  %f y:=  %f z:=  %f", avg_pose.position.x, avg_pose.position.y, avg_pose.position.z);

            path.push_back(avg_pose);
            detection_ += filtered;
            min_distance += forward_increment;
            real_points = 0;
            x_acum = 0; y_acum = 0; z_acum = 0; 
        }

        // if(std::hypot(path.back().position.x - goal.position.x, path.back().position.y - goal.position.y) < min_error_) {
        //     path.push_back(goal);
        // }

        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(detection_, cloud_out);
        cloud_out.header.frame_id = tf_info.robot_frame;
        cloud_out.header.stamp = get_node()->now();
        detection_pub_->publish(cloud_out);

        std::reverse(path.begin(), path.end());

        if (path.empty()) {path.push_back(goal);}
        return path;

    }

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyPlanner, easynav::PlannerMethodBase)