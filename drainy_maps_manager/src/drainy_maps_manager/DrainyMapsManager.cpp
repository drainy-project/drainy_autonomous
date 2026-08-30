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
        node->declare_parameter(plugin_name + ".filter_min", filter_min_);
        node->declare_parameter(plugin_name + ".filter_max", filter_max_);
        node->declare_parameter(plugin_name + ".max_lenght", max_lenght_);
        node->declare_parameter(plugin_name + ".resolution", resolution_);

        node->get_parameter(plugin_name + ".map_path", map_path_);
        node->get_parameter(plugin_name + ".map_topic", map_topic_);
        node->get_parameter(plugin_name + ".filter_min", filter_min_);
        node->get_parameter(plugin_name + ".filter_max", filter_max_);
        node->get_parameter(plugin_name + ".resolution", resolution_);

        drainy_map_.initialize(max_lenght_,max_lenght_,resolution_, origin_x_, origin_y_);

        occ_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
            rclcpp::QoS(1).transient_local().reliable());

        laser_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/scan", 
            rclcpp::QoS(1).transient_local().reliable());

        cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node->get_fully_qualified_name() + std::string("/") + plugin_name + "/cloud_filtered", 
            rclcpp::QoS(1).transient_local().reliable());

        incoming_pc2_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_,
            rclcpp::QoS(100),
            [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_msg) {

                pc2_map_msg_ = *raw_msg;
                auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(pc2_map_msg_, *cloud);
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(static_cast<float>(filter_min_), 
                    static_cast<float>(filter_max_));
                pass.filter(*cloud_filtered);
                pcl::toROSMsg(*cloud_filtered, *msg);
                msg->header.frame_id = tf_info.map_frame;
                msg->header.stamp = pc2_map_msg_.header.stamp;
                cloud_pub_->publish(*msg);

                 // build laserscan output
                auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                scan_msg->header.stamp = msg->header.stamp;
                scan_msg->header.frame_id = tf_info.map_frame;
                scan_msg->angle_min = -M_PI;
                scan_msg->angle_max = M_PI;
                scan_msg->angle_increment = M_PI / 180.0;
                scan_msg->time_increment = 0.0;
                scan_msg->scan_time = 1.0 / 30.0;
                scan_msg->range_min = 0.0;
                scan_msg->range_max = std::numeric_limits<double>::max();

                // determine amount of rays to create
                uint32_t ranges_size = std::ceil(
                    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

                // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
                scan_msg->ranges.assign(ranges_size, scan_msg->range_max + 1.0);

                // Iterate through pointcloud
                for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                    iter_y(*msg, "y"), iter_z(*msg, "z");
                    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
                {
                    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                    RCLCPP_INFO(
                        get_node()->get_logger(),
                        "rejected for nan in point(%f, %f, %f)\n",
                        *iter_x, *iter_y, *iter_z);
                    continue;
                    }

                    if (*iter_z > (filter_max_+5.0) || *iter_z < (filter_min_-5.0)) {
                    RCLCPP_INFO(
                        get_node()->get_logger(),
                        "rejected for height %f not in range (%f, %f)\n",
                        *iter_z, -5.0, 5.0);
                    continue;
                    }

                    double range = hypot(*iter_x, *iter_y);
                    if ((0.0 > range) && (range < 500.0)) {
                    RCLCPP_INFO(
                        get_node()->get_logger(),
                        "rejected for range %f out of bounds. Point: (%f, %f, %f)",
                        range, *iter_x, *iter_y, *iter_z);
                    continue;
                    }

                    double angle = atan2(*iter_y, *iter_x);
                    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
                    RCLCPP_INFO(
                        get_node()->get_logger(),
                        "rejected for angle %f not in range (%f, %f)\n",
                        angle, scan_msg->angle_min, scan_msg->angle_max);
                    continue;
                    }

                    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
                    if (range < scan_msg->ranges[index]) {
                        scan_msg->ranges[index] = range;
                    }
                }
                laser_pub_->publish(*scan_msg);
                drainy_map_.from_laser_scan(*scan_msg);
                drainy_map_.to_occupancy_grid(occ_map_msg_);
                occ_map_msg_.header.frame_id = tf_info.map_frame;
                occ_map_msg_.header.stamp = get_node()->now();
                occ_map_pub_->publish(occ_map_msg_);
                //drainy_map_.print(false);

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
        if (!nav_state.has("robot_pose") && map_set_) {
            RCLCPP_INFO(get_node()->get_logger(), "No Robot Pose. No Map Saved");
            return;
        }
    }


} // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyMapsManager, easynav::MapsManagerBase)