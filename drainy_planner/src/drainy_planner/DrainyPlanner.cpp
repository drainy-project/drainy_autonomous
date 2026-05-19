#include <memory>
#include <vector>

#include "drainy_planner/DrainyPlanner.hpp"


namespace easynav
{
    DrainyPlanner::DrainyPlanner(){}

    DrainyPlanner::~DrainyPlanner(){}

    void DrainyPlanner::on_initialize()
    {

        auto node = get_node();
        const std::string & plugin_name = this->get_plugin_name();

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
        const auto & goal = goals.goals.front().pose;
        const auto & perceptions = nav_state.get<PointPerceptions>("points");
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

        auto poses = get_poses(robot_pose.pose.pose, goal);

        // nav_msgs::msg::Path path = nav_state.get<nav_msgs::msg::Path>("path");
        // if (path.poses.empty()) {
        //     // If the path is empty, stop the robot
        //     current_path_.poses.clear();
        //     current_path_.header.stamp = get_node()->now();
        //     current_path_.header.frame_id = goals.header.frame_id;
        //     for (const auto & pose : poses) {
        //     geometry_msgs::msg::PoseStamped pose_stamped;
        //     pose_stamped.header.frame_id = goals.header.frame_id;
        //     pose_stamped.header.stamp = current_path_.header.stamp;
        //     pose_stamped.pose = pose;
        //     current_path_.poses.push_back(pose_stamped);
        //     }
        // }

        // path_pub_->publish(current_path_);


        const auto & filtered = PointPerceptionsOpsView(perceptions)
        .filter({3.0, -5.0, -5.0}, {3.3, 5.0, 5.0})
        .fuse(tf_info.map_frame)
        // .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
        // .collapse({0.1, NAN, NAN})
        .downsample(0.1)
        .as_points();


        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(filtered, cloud_out);
        cloud_out.header.frame_id = tf_info.map_frame;
        cloud_out.header.stamp = get_node()->now();
        detection_pub_->publish(cloud_out);
        
    }

    std::vector<geometry_msgs::msg::Pose> DrainyPlanner::get_poses(
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & goal)
    {
        std::vector<geometry_msgs::msg::Pose> path;

        if (path.empty()) {path.push_back(goal);}
        return path;

    }

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyPlanner, easynav::PlannerMethodBase)