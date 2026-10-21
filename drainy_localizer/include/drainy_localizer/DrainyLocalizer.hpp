// Copyright 2026 Intelligent Robotics Lab

/// \file
/// \brief Declaration of the DrainyLocalizer class.

#ifndef DRAINY__LOCALIZER__DRAINYLOCALIZER_HPP_
#define DRAINY__LOCALIZER__DRAINYLOCALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace easynav
{

/**
 * @brief EasyNav localizer plugin for the Drainy robot.
 *
 * Takes the robot pose from an external odometry source and exposes it to the
 * navigation system as "robot_pose". It also publishes the map -> odom and
 * odom -> base frame transforms, a copy of the odometry and the trajectory
 * followed by the robot.
 */
class DrainyLocalizer : public LocalizerMethodBase
{
    public:

    /**
     * @brief Constructor.
     *
     * Registers a printer in NavState to display nav_msgs::msg::Odometry values.
     */
    DrainyLocalizer();

    /**
     * @brief Destructor.
     */
    ~DrainyLocalizer();

    /**
     * @brief Initializes the localizer plugin.
     *
     * Declares and reads the initial pose parameters, creates the odometry
     * subscription, the trajectory and odometry publishers and the TF broadcaster,
     * and stores the initial pose.
     */
    void on_initialize() override;

    /**
     * @brief Updates the robot pose in the real-time loop.
     *
     * On the first call, publishes the initial map -> odom transform. Afterwards,
     * stores the latest odometry in the navigation state as "robot_pose".
     *
     * @param nav_state Shared navigation state where the robot pose is stored.
     */
    void update_rt(NavState & nav_state) override;

    /**
     * @brief Updates the robot pose in the non real-time loop.
     *
     * Same as update_rt(), and also publishes the odom -> base frame and
     * map -> odom transforms and appends the current pose to the published
     * trajectory.
     *
     * @param nav_state Shared navigation state where the robot pose is stored.
     */
    void update(NavState & nav_state) override;

    /**
     * @brief Returns the latest robot pose received from the odometry.
     * @return Transform built from the position and orientation of the last odometry message.
     */
    tf2::Transform get_pose(void);

    protected:

    /**
     * @brief Publishes the map -> odom transform to TF and to the EasyNav TF buffer.
     * @param map2odom Transform from the map frame to the odom frame.
     */
    void publish_odom_TF(const tf2::Transform & map2odom);

    /**
     * @brief Publishes the odom -> robot base frame transform to TF and to the EasyNav TF buffer.
     * @param map2bf Transform from the odom frame to the robot base frame.
     */
    void publish_bf_TF(const tf2::Transform & map2bf);

    /**
     * @brief Publishes the initial map -> odom transform from init_pose_ and
     *        marks the odometry as initialized.
     */
    void init_odom(void);

    /**
     * @brief Prints the translation and rotation (quaternion) of a transform to stderr.
     * @param tf Transform to print.
     */
    void printTransform(const tf2::Transform & tf);

    /**
     * @brief Sets the initial pose of the robot.
     * @param x Initial X position (m).
     * @param y Initial Y position (m).
     * @param z Initial Z position (m).
     * @param yaw Initial yaw angle (rad).
     */
    void set_init_pose(double x, double y, double z, double yaw);

    /**
     * @brief Builds an odometry message from the latest robot pose.
     *
     * The message is expressed in the odom frame with the robot base frame as
     * child, and its twist is set to zero. Also updates odom_tf_.
     *
     * @return Odometry message with the current robot pose.
     */
    nav_msgs::msg::Odometry get_odom(void);

    /// Subscription to the external odometry source.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    /// Trajectory followed by the robot, as the list of poses received so far.
    nav_msgs::msg::Path trajectory_;

    /// Publisher of the robot trajectory (trajectory_).
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    /// Publisher that republishes the received odometry.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    /// Last odometry message received.
    nav_msgs::msg::Odometry odom_msg_;

    /// Initial pose of the robot, read from the plugin parameters.
    geometry_msgs::msg::Pose init_pose_;

    /// Latest robot pose as a transform, updated in get_odom().
    tf2::Transform odom_tf_{tf2::Transform::getIdentity()};

    /// Timestamp of the last odometry message, used to stamp the published transforms.
    rclcpp::Time last_input_time_;

    /// Broadcaster used to publish transforms to TF.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// True once the initial map -> odom transform has been published.
    bool initialized_odom_{false};

};

} // namespace easynav

#endif // DRAINY__LOCALIZER__DRAINYLOCALIZER_HPP_
