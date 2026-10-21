// Copyright 2026 Intelligent Robotics Lab

/// \file
/// \brief Declaration of the DrainyController class.

#ifndef DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_
#define DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_system/GoalManager.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace easynav
{

/**
 * @brief EasyNav controller plugin for the Drainy robot.
 *
 * Computes velocity commands (linear x, y, z and angular z) to drive the robot
 * towards the last pose of the planned path. It uses point perceptions to detect
 * nearby obstacles in the horizontal plane and in the vertical direction, adding
 * offsets to the commands to avoid imminent collisions.
 */
class DrainyController : public ControllerMethodBase
{
    public:

    /**
     * @brief Default constructor.
     */
    DrainyController();

    /**
     * @brief Destructor.
     */
    ~DrainyController();

    /**
     * @brief Initializes the controller plugin.
     *
     * Declares and reads the plugin parameters (gains and safety radius) and
     * creates the publishers for the horizontal and vertical detection clouds.
     */
    void on_initialize() override;

    /**
     * @brief Computes the velocity command in the real-time control loop.
     *
     * Reads the path, the robot pose and the point perceptions from the
     * navigation state, publishes the filtered detection clouds, estimates the
     * available height, and writes the resulting "cmd_vel" and "height" entries
     * back into the navigation state. A zero velocity is commanded when the
     * navigation is IDLE or the path is empty. If the heading error exceeds
     * yaw_limit_, the robot only rotates.
     *
     * @param nav_state Shared navigation state used to read inputs and store outputs.
     */
    void update_rt(NavState & nav_state) override;

    protected:

    /// Last velocity command computed by the controller.
    geometry_msgs::msg::TwistStamped cmd_vel_;

    /// Point cloud of the obstacles detected in the horizontal plane around the robot.
    sensor_msgs::msg::PointCloud2 cloud_h_msg_;

    /// Point cloud of the obstacles detected in front of the robot along the vertical axis.
    sensor_msgs::msg::PointCloud2 cloud_v_msg_;

    /// Publisher of the horizontal detection cloud (cloud_h_msg_).
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_h_pub_;

    /// Publisher of the vertical detection cloud (cloud_v_msg_).
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_v_pub_;

    private:

    /// Proportional gain applied to the distance error to compute the linear X velocity.
    double x_gain_{1.0};

    /// Proportional gain applied to the lateral centering error to compute the linear Y velocity.
    double y_gain_{1.0};

    /// Proportional gain applied to the Z error to compute the linear Z velocity.
    double z_gain_{1.0};

    /// Proportional gain applied to the yaw error to compute the angular Z velocity.
    double yaw_gain_{0.8};

    /// Maximum range (m) of the points used for obstacle detection. Tuned for the RealSense camera.
    double detection_limit_{3.5}; // For realsense

    /// Horizontal distance (m) below which an obstacle is considered an imminent collision.
    double safety_radius_{1.0};

    /// Vertical distance (m) below which an obstacle is considered an imminent collision.
    double safety_vertical_{0.3};

    /// Maximum linear velocity (m/s) allowed on each axis.
    double vel_lineal_max_{0.5};

    /// Maximum angular velocity (rad/s) allowed around the Z axis.
    double vel_angular_max_{1.0};

    /// Maximum yaw error (rad) allowed before the robot stops translating and only rotates.
    double yaw_limit_{0.1};

    /// Distance (m) to the goal below which the robot switches to final approach mode.
    double convergence_limit_{2.0};

    /// Estimated lateral error (m) of the robot with respect to the center of the free space.
    double ey_medio_{0.0};

};

} // namespace easynav

#endif // DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_
