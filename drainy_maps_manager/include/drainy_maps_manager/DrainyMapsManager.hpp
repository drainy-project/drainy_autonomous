#ifndef DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_
#define DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_navmap_maps_manager/filters/NavMapFilter.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "pluginlib/class_loader.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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
        sensor_msgs::msg::PointCloud2 pc2_map_msg_;
        navmap_ros_interfaces::msg::NavMap navmap_msg_;

        // mapa modificado
        ::navmap::NavMap navmap_;

        rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr navmap_pub_;
        rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr layer_updates_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

        std::unique_ptr<pluginlib::ClassLoader<navmap::NavMapFilter>> navmap_filters_loader_;
        std::vector<std::shared_ptr<navmap::NavMapFilter>> navmap_filters_;

        bool map_set_{false};
    }; 

} // namesapce easynav

#endif // DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_