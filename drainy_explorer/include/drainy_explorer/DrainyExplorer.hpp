// Copyright 2026 Intelligent Robotics Lab

/// \file
/// \brief Declaration of the DrainyExplorer node.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include "easynav_system/GoalManagerClient.hpp"

#include "drainy_common/DrainyMap.hpp"

using namespace std::chrono_literals;

namespace easynav
{

/**
 * @brief Builds a quaternion message from a yaw angle.
 * @param angle Rotation around the Z axis, in radians.
 * @return Quaternion with zero roll and pitch and the given yaw.
 */
inline geometry_msgs::msg::Quaternion setYaw(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  return tf2::toMsg(q);
}

/**
 * @brief ROS 2 node that autonomously explores the environment.
 *
 * Receives the occupancy map and the robot pose, looks for free cells along the
 * forward, left, right and backward directions of the robot, and sends them as
 * navigation goals through the EasyNav GoalManager. The exploration is driven
 * by a state machine executed periodically by a timer.
 */
class DrainyExplorer : public rclcpp::Node
{
    public:
    RCLCPP_SMART_PTR_DEFINITIONS(DrainyExplorer)

    /**
     * @brief Constructor.
     *
     * Creates the cycle timer and the subscriptions to the map and to the
     * navigation control topics.
     *
     * @param options Node options.
     */
    explicit DrainyExplorer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Default destructor.
     */
    ~DrainyExplorer() = default;

    private:
    /**
     * @brief States of the exploration state machine.
     */
    enum class ExplorerState {
      IDLE,           ///< Waiting to start; initializes the node and looks for the first goal.
      EXPLORING,      ///< A goal has been sent and the robot is navigating towards it.
      FINISHED,       ///< No more goals were found; the exploration is completed.
      ERROR,          ///< Navigation failed, was rejected or was cancelled.
      DO_AT_WAYPOINT  ///< A goal has been reached; looks for the next one.
    };

    /**
     * @brief Search direction used to look for free cells around the robot.
     */
    struct SearchDirection {
      /// Step along the X axis of the map, in cells.
      int dx;
      /// Step along the Y axis of the map, in cells.
      int dy;
      /// Yaw offset (rad) added to the current robot yaw for the goal orientation.
      double yaw_offset;
      /// Human-readable name of the direction, used for logging.
      std::string name;
    };

    /**
     * @brief Initializes the exploration: sets the frame of the goals message
     *        and stores the current pose as the home position.
     */
    void initialize();

    /**
     * @brief Executes one iteration of the exploration state machine.
     *
     * Called periodically by timer_.
     */
    void cycle();

    /**
     * @brief Looks for a new goal and sends it to the GoalManager.
     *
     * Directions are checked in order: forward, left, right and backward. The
     * first free cell found is sent as a goal.
     *
     * @return true if a goal was found and sent, false otherwise.
     */
    bool find_goal();

    /**
     * @brief Stores the current robot pose as the home position.
     */
    void set_home();

    /**
     * @brief Searches for a free cell along a given direction from the robot position.
     *
     * The search starts at max_long_ meters from the robot and moves towards it,
     * so the farthest free cell is selected.
     *
     * @param direction Direction in which to search.
     * @param goal Output vector filled with the goal as {x, y, yaw}.
     * @return true if a free cell was found, false otherwise.
     */
    bool search_goal_in_direction(const SearchDirection& direction, std::vector<double>& goal);

    /// Current state of the exploration state machine.
    ExplorerState state_ {ExplorerState::IDLE};

    /// True once the GoalManager client has been created and the node initialized.
    bool initialized_ {false};

    /// True once the exploration has finished, to avoid restarting it.
    bool completed_ {false};

    /// Number of cycles spent waiting for the goal to be accepted.
    size_t send_retries_ {0};

    /// Maximum number of cycles to wait for the goal to be accepted before resending it.
    const size_t max_retries_ {3};

    /// Type of the last navigation control message received from the GoalManager.
    uint last_control_type_ {0};

    /// Maximum distance (m) from the robot at which goals are searched.
    double max_long_{5.0};

    /// Occupancy map used to look for free cells.
    DrainyMap drainy_map_;

    /// Subscription to the occupancy map published by the maps manager.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr incoming_map_sub_;

    /// Subscription to the navigation control topic, used to get the current robot pose.
    rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr incoming_control_sub_;

    /// Frame in which goals and the home position are expressed.
    std::string frame_id_{"map"};

    /// List of goals sent to the GoalManager.
    nav_msgs::msg::Goals goals_;

    /// Goal currently being navigated to. Currently unused.
    geometry_msgs::msg::PoseStamped current_goal_;

    /// Home position, set to the robot pose when the exploration starts.
    geometry_msgs::msg::PoseStamped current_home_;

    /// Latest known robot pose.
    geometry_msgs::msg::PoseStamped current_pose_;

    /// Client used to send goals to the EasyNav GoalManager and monitor their state.
    GoalManagerClient::SharedPtr gm_client_;

    /// Time at which the robot reached the last waypoint.
    rclcpp::Time pause_start_time_;

    /// Time to wait at each waypoint before looking for the next goal. Currently unused.
    rclcpp::Duration pause_duration_ = rclcpp::Duration::from_seconds(2.0);

    /// Timer that periodically calls cycle().
    rclcpp::TimerBase::SharedPtr timer_;

};

} // namespace easynav
