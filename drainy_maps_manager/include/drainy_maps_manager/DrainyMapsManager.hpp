// Copyright 2026 Intelligent Robotics Lab

/// \file
/// \brief Declaration of the DrainyMapsManager class.

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
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "drainy_common/DrainyMap.hpp"

namespace easynav
{
    /**
     * @brief EasyNav maps manager plugin for the Drainy robot.
     *
     * Builds a 2D occupancy map (DrainyMap) from an incoming point cloud map.
     * The cloud is filtered to keep only the points close to the current robot
     * altitude, and the result is used to update the occupancy grid, which is
     * published and stored in the navigation state as "map". It also offers a
     * service to save the received point cloud map to disk.
     */
    class DrainyMapsManager : public MapsManagerBase
    {
        public:
        /**
         * @brief Default constructor.
         */
        DrainyMapsManager();

        /**
         * @brief Destructor.
         */
        ~DrainyMapsManager();

        /**
         * @brief Initializes the maps manager plugin.
         *
         * Declares and reads the plugin parameters, initializes the occupancy
         * map, and creates the publishers, the point cloud map subscription and
         * the save map service. Each received cloud is filtered by altitude,
         * published, and used to update and publish the occupancy grid.
         */
        virtual void on_initialize() override;

        /**
         * @brief Updates the robot pose and stores the map in the navigation state.
         *
         * Reads "robot_pose" from the navigation state, when available, and sets
         * the current DrainyMap as "map".
         *
         * @param nav_state Shared navigation state used to read the pose and store the map.
         */
        virtual void update(NavState & nav_state) override;

        protected:
        /// Directory where maps are saved by the save map service.
        std::string map_path_ {"/tmp/drainy_maps/"};

        /// Topic of the incoming point cloud map.
        std::string map_topic_ {"/genz/local_map"};

        private:

        /// 2D occupancy map built from the filtered point clouds.
        DrainyMap drainy_map_;

        /// Last point cloud map received, before filtering.
        sensor_msgs::msg::PointCloud2 pc2_map_msg_;

        /// Occupancy grid message generated from drainy_map_.
        nav_msgs::msg::OccupancyGrid occ_map_msg_;

        /// Latest robot pose, read from the navigation state.
        geometry_msgs::msg::Pose pose_;

        /// Publisher of the occupancy grid map.
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;

        /// Publisher of the point cloud filtered by altitude.
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

        /// Subscription to the incoming point cloud map (map_topic_).
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;

        /// Service that saves the last received point cloud map as a PCD file in map_path_.
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

        /// Lower altitude limit, as a factor of the robot altitude, for the point cloud filter.
        double filter_min_ {0.95};

        /// Upper altitude limit, as a factor of the robot altitude, for the point cloud filter.
        double filter_max_ {1.05};

        /// Resolution of the occupancy map (m / cell).
        double resolution_{0.25};

        /// Width and height of the occupancy map (m).
        double max_lenght_{50};

        /// Radius (m) around the robot considered free when updating the map.
        double confidence_lenght_{1.0};

        /// X coordinate of the map center in the world frame (m).
        double origin_x_{0.0};

        /// Y coordinate of the map center in the world frame (m).
        double origin_y_{0.0};

        /// True once the map has been set. Currently never set to true.
        bool map_set_{false};
    };

} // namesapce easynav

#endif // DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_
