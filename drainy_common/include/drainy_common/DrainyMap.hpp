// Copyright 2026 Intelligent Robotics Lab

/// \file
/// \brief Declaration of the DrainyMap type.

#ifndef DRAINY_COMMON__DRAINYMAP_HPP_
#define DRAINY_COMMON__DRAINYMAP_HPP_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace easynav
{

/**
 * @brief 2D occupancy grid map built from point cloud observations.
 *
 * The map stores one occupancy value per cell in row-major order, following the
 * nav_msgs::msg::OccupancyGrid convention (0 = free, 100 = occupied, -1 = unknown).
 * The origin (origin_x_, origin_y_) is the metric position of the map center in
 * the world frame.
 */
class DrainyMap
{
public:
  /**
   * @brief Default constructor.
   *
   * Creates an empty map (0 x 0 cells) with a resolution of 1.0 m/cell and the
   * origin at (0, 0).
   */
  DrainyMap();

  /**
   * @brief Initializes the map dimensions, resolution and origin.
   *
   * All cells are reset to the unknown value.
   *
   * @param m_width Width of the map in meters.
   * @param m_height Height of the map in meters.
   * @param resolution Size of each cell in meters per cell.
   * @param origin_x X coordinate of the map center in the world frame (m).
   * @param origin_y Y coordinate of the map center in the world frame (m).
   */
  void initialize(
    double m_width, double m_height, double resolution, double origin_x, double origin_y);

  /**
   * @brief Updates the map with the points of a PointCloud2 message.
   *
   * For each valid (non-NaN) point, the cells along the ray from the robot
   * position to the point are marked as free (0), and the cell containing the
   * point is marked as occupied (100).
   *
   * @param msg Point cloud expressed in the world frame.
   * @param pose Current robot pose in the world frame, used as the ray origin.
   * @param safe_lenght Radius (m) around the robot considered free. Currently unused.
   */
  void from_pc2(const sensor_msgs::msg::PointCloud2 msg,
    geometry_msgs::msg::Pose pose,
    double safe_lenght);

  /**
   * @brief Updates a nav_msgs::msg::OccupancyGrid message from the DrainyMap contents.
   *
   * Cell values are copied as they are stored (0 = free, 100 = occupied, -1 = unknown).
   * The message origin is set to the lower-left corner of the map.
   *
   * If the grid dimensions do not match, the message data is resized automatically.
   *
   * @param grid_msg The occupancy grid message to fill or update.
   */
  void to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg) const;

  /**
   * @brief Initializes the map from a nav_msgs::msg::OccupancyGrid message.
   *
   * Dimensions, resolution and origin are taken from the message info, and the
   * cell values are copied from the message data.
   *
   * @param grid_msg The occupancy grid message to read from.
   */
  void from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid_msg);

  /**
  * @brief Saves the map to a file, including metadata and cell data.
  * @param path Path to the output file.
  * @return true if the file was written successfully, false otherwise.
  */
  //bool save_to_file(const std::string & path) const;

  /**
   * @brief Prints the map metadata (and optionally the cell data) to stderr.
   * @param view_data If true, also prints every cell with its index, metric
   *        position and value.
   */
  void print(bool view_data) const;

  /**
   * @brief Returns the map width.
   * @return Number of cells along the X axis.
   */
  size_t width() const {return width_;}

  /**
   * @brief Returns the map height.
   * @return Number of cells along the Y axis.
   */
  size_t height() const {return height_;}

  /**
   * @brief Returns the map resolution.
   * @return Size of each cell in meters per cell.
   */
  double resolution() const {return resolution_;}

  /**
   * @brief Returns the value stored in a cell.
   * @param index Linear (row-major) index of the cell: y * width + x.
   * @return Occupancy value of the cell, or 0 if the index is out of bounds.
   */
  uint8_t get_data(int index) const;

  /**
   * @brief Converts cell coordinates to metric coordinates, taking the origin
   *        as the lower-left corner of the map.
   * @param x Cell index along the X axis.
   * @param y Cell index along the Y axis.
   * @return Metric coordinates (x, y) of the cell, in meters.
   */
  std::pair<double, double> cell_to_metric(int x, int y) const;

  /**
   * @brief Converts cell coordinates to world-frame metric coordinates, taking
   *        the origin as the center of the map.
   * @param x Cell index along the X axis.
   * @param y Cell index along the Y axis.
   * @return World-frame coordinates (x, y) of the cell, in meters.
   */
  std::pair<double, double> world_cell_to_metric(int x, int y) const;

  /**
   * @brief Converts metric coordinates to cell coordinates, taking the origin
   *        as the lower-left corner of the map.
   * @param mx X coordinate in meters.
   * @param my Y coordinate in meters.
   * @return Cell indices (x, y). No bounds checking is performed.
   */
  std::pair<int, int> metric_to_cell(double mx, double my) const;

  /**
   * @brief Converts world-frame metric coordinates to cell coordinates, taking
   *        the origin as the center of the map.
   * @param mx X coordinate in the world frame, in meters.
   * @param my Y coordinate in the world frame, in meters.
   * @return Cell indices (x, y). No bounds checking is performed.
   */
  std::pair<int, int> world_metric_to_cell(double mx, double my) const;

  /**
   * @brief Computes the cells crossed by a straight line between two cells
   *        using Bresenham's line algorithm.
   * @param gx X index of the start cell.
   * @param gy Y index of the start cell.
   * @param px X index of the end cell.
   * @param py Y index of the end cell.
   * @return Ordered list of cells (x, y) from start to end, both included.
   */
  std::vector<std::pair<int, int>> ray_trace(int gx, int gy, int px, int py) const;

private:
  /// Number of cells along the X axis.
  size_t width_;

  /// Number of cells along the Y axis.
  size_t height_;

  /// Size of each cell (m / cell).
  double resolution_;

  /// X coordinate of the map center in the world frame (m).
  double origin_x_;

  /// Y coordinate of the map center in the world frame (m).
  double origin_y_;

  /// Occupancy values of the cells, stored in row-major order (index = y * width_ + x).
  std::vector<uint8_t> data_;
};

}  // namespace easynav

#endif  // DRAINY_COMMON__DRAINYMAP_HPP_
