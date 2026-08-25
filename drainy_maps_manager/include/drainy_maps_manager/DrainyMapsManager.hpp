#ifndef DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_
#define DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "pluginlib/class_loader.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
#include "nav_msgs/msg/odometry.hpp"

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

        sensor_msgs::msg::PointCloud2 out_map_msg_;

        navmap_ros_interfaces::msg::NavMap navmap_msg_;

        ::navmap::NavMap navmap_;

        rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr navmap_pub_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_map_pub_;
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

        double resoultion_{0.25};

        bool map_set_{false};
    }; 

} // namesapce easynav

#endif // DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_