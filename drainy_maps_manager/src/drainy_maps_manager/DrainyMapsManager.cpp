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

        incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_,
            rclcpp::QoS(100),
            [&](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

            // navmap_ros::BuildParams params;
            // navmap_ = navmap_ros::from_pointcloud2(*msg, navmap_msg_, params);


            // navmap_msg_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
            // navmap_msg_.header.stamp = this->get_node()->now();
            // navmap_pub_->publish(navmap_msg_);
            });

        savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/savemap",
            [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
            (void)request;
            (void)response;
            // navmap_ros::io::save_to_file(navmap_, "/tmp/map.navmap");
            // ToDo
            });

    }

    void DrainyMapsManager::update(NavState & nav_state)
    {

    }


} // namespace easynav