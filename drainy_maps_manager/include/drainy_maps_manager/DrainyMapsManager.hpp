#ifndef DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_
#define DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"

#include "pluginlib/class_loader.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include <pcl/filters/passthrough.h>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "drainy_common/DrainyMap.hpp"

namespace easynav
{
    class DrainyMapsManager : public MapsManagerBase
    {
        public:
        DrainyMapsManager();
        ~DrainyMapsManager();

        virtual void on_initialize() override;

        virtual void update(NavState & nav_state) override;

        protected:
        std::string map_path_ {"/tmp/drainy_maps/"};
        std::string map_topic_ {"/genz/local_map"};

        private:

        DrainyMap drainy_map_;

        sensor_msgs::msg::PointCloud2 pc2_map_msg_;
        nav_msgs::msg::OccupancyGrid occ_map_msg_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

        double filter_min_ {0.75};
        double filter_max_ {1.25};
        double resoultion_{0.25};
        bool map_set_{false};
    }; 

} // namesapce easynav

#endif // DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_