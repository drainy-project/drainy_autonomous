// Copyright 2026 Intelligent Robotics Lab

/// \file
/// \brief Declaration of the DrainyPlanner class implementing A* path planning on a DrainyMap.

#ifndef DRAINY__PLANNER__DRAINYLOCALIZER_HPP_
#define DRAINY__PLANNER__DRAINYLOCALIZER_HPP_

#include <queue>
#include <unordered_map>
#include <cmath>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "drainy_common/DrainyMap.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @brief EasyNav planner plugin implementing the A* algorithm on a DrainyMap grid.
 *
 * Computes a path from the robot pose to the first goal of the navigation
 * state, at the height given by the controller, and stores it as "path".
 */
class DrainyPlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Registers a printer in NavState to display nav_msgs::msg::Path values
   * (number of poses, length and average height).
   */
  explicit DrainyPlanner();

  /**
   * @brief Initializes the planner.
   *
   * Declares and reads the plugin parameters (robot radius, clearance distance
   * and replanning) and creates the path publisher.
   */
  virtual void on_initialize() override;

  /**
   * @brief Updates the planner by computing a new path.
   *
   * Uses the current navigation state (robot pose, goals, map and height) to
   * generate a path to the first goal with the A* algorithm. The path is
   * published and stored in the navigation state as "path". An empty path is
   * stored when there are no goals.
   *
   * @param nav_state The current navigation state (contains odometry and goal information).
   */
  void update(NavState & nav_state) override;

protected:
  /// If true, the path is recomputed while navigating. Currently read but not used.
  bool replanning_{false};

  /// Radius of the robot (m) used for collision checking.
  double robot_radius_;

  /// Minimum clearance distance from obstacles (m).
  double clearance_distance_;

  /// The last computed path.
  nav_msgs::msg::Path current_path_;

  /// Publisher for the computed navigation path.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /**
   * @brief Runs the A* algorithm to compute a path.
   *
   * If no path is found, a path containing only the goal pose is returned.
   *
   * @param map The occupancy map used for path planning.
   * @param start The starting pose in world coordinates.
   * @param goal The target pose in world coordinates.
   * @param resolution The cell resolution of the map (in meters).
   * @param height Height (Z coordinate, in meters) assigned to every pose of the path.
   * @return A sequence of poses representing the planned path, all with the goal orientation.
   */
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const DrainyMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double resolution,
    double height);

  /**
   * @brief Checks whether a map cell is free, considering a clearance area.
   *
   * This function verifies if a cell and its surrounding cells (within the
   * specified clearance radius) are free of obstacles.
   *
   * @param map The occupancy map to query.
   * @param cx The x-coordinate of the cell.
   * @param cy The y-coordinate of the cell.
   * @param clearance_cells The clearance radius expressed in number of cells.
   * @return true if the cell and its clearance area are free, false otherwise.
   */
  // bool isFreeWithClearance(
  //   const SimpleMap & map,
  //   int cx, int cy,
  //   double clearance_cells);
};

}  // namespace easynav

#endif // DRAINY__PLANNER__DRAINYLOCALIZER_HPP_
