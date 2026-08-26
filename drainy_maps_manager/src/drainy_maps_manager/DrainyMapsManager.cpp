#include "drainy_maps_manager/DrainyMapsManager.hpp"

namespace easynav
{
    DrainyMapsManager::DrainyMapsManager(){};

    DrainyMapsManager::~DrainyMapsManager(){};

    void DrainyMapsManager::on_initialize()
    {
        auto node = get_node();
        const auto & plugin_name = get_plugin_name();

        std::string package_name;

        node->declare_parameter(plugin_name + ".map_path", map_path_);
        node->declare_parameter(plugin_name + ".map_topic", map_topic_);

        node->get_parameter(plugin_name + ".map_path", map_path_);
        node->get_parameter(plugin_name + ".map_topic", map_topic_);

        navmap_pub_ = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/navmap",
            rclcpp::QoS(1).transient_local().reliable());

        pc2_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
            rclcpp::QoS(1).transient_local().reliable());

        incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_,
            rclcpp::QoS(100),
            [&](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

                pc2_map_msg_ = *msg;

                navmap_ros::BuildParams params;
                params.resolution = 0.5f;
                navmap_ = navmap_ros::from_pointcloud2(*msg, navmap_msg_, params);
                map_set_ = true;
                navmap_msg_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
                navmap_msg_.header.stamp = this->get_node()->now();
                navmap_pub_->publish(navmap_msg_);
            });

        savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
                {
                (void)request;
                rclcpp::Time now = get_node()->now();
                std::string now_str = std::to_string(now.nanoseconds());
                std::string tmp_pc2_map = map_path_ + "map_" + now_str + ".pcd";

                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(pc2_map_msg_, cloud);
                pcl::io::savePCDFileASCII(tmp_pc2_map, cloud);

                response->success = true;
                response->message = " [Drainy Maps Mapanager] : Map successfully saved to: " + tmp_pc2_map;
            });

    }

    void DrainyMapsManager::update(NavState & nav_state)
    {
        if (!nav_state.has("map.navmap") || map_set_) {
            nav_state.set("map.navmap", navmap_);
        }

        if (!nav_state.has("robot_pose") && map_set_) {
            RCLCPP_INFO(get_node()->get_logger(), "No Robot Pose. No Map Saved");
            return;
        }
        
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

        const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

        tf2::Transform tf;
        tf2::fromMsg(robot_pose.pose.pose, tf);

        pcl::PointCloud<pcl::PointXYZ> pcl_in;
        pcl::fromROSMsg(pc2_map_msg_, pcl_in);

        pcl::PointCloud<pcl::PointXYZ> pcl_out;
        pcl_out.reserve(pcl_in.points.size());

        for (const auto & p : pcl_in.points) {
            if(!std::isnan(p.x) || !std::isnan(p.y) || !std::isnan(p.z)){
                tf2::Vector3 ps(p.x, p.y, p.z);
                tf2::Vector3 p_map = tf * ps;

                pcl_out.push_back(pcl::PointXYZ(
                    static_cast<float>(p_map.x()),
                    static_cast<float>(p_map.y()),
                    static_cast<float>(p_map.z())));
            } 
        }

        pcl::toROSMsg(pcl_out, out_map_msg_);
        out_map_msg_.header.frame_id = tf_info.map_frame;
        out_map_msg_.header.stamp = pc2_map_msg_.header.stamp;
        pc2_map_pub_->publish(out_map_msg_);

    }


} // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyMapsManager, easynav::MapsManagerBase)