// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Declaration of the SimpleMap type.

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

class DrainyMap
{
public:

  DrainyMap();

  void initialize(
    double m_width, double m_height, double resolution, double origin_x, double origin_y);

  void from_pc2(const sensor_msgs::msg::PointCloud2 msg, 
    geometry_msgs::msg::Pose pose,
    double safe_lenght);
  /**
   * @brief Updates a nav_msgs::msg::OccupancyGrid message from the SimpleMap contents.
   *
   * Cells are mapped as follows:
   * - true  -> 100 (occupied)
   * - false -> 0 (free)
   *
   * If the grid dimensions do not match, the message is rdata_esized automatically.
   *
   * @param grid_msg The occupancy grid message to fill or update.
   */
  void to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg) const;

  void from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid_msg);

  /**
  * @brief Saves the map to a file, including metadata and cell data.
  * @param path Path to the output file.
  * @return true if the file was written successfully, false otherwise.
  */
  //bool save_to_file(const std::string & path) const;

  void print(bool view_data) const;

  size_t width() const {return width_;}
  size_t height() const {return height_;}
  double resolution() const {return resolution_;}
  uint8_t get_data(int index) const;
  
  std::pair<double, double> cell_to_metric(int x, int y) const;
  std::pair<double, double> world_cell_to_metric(int x, int y) const;
  std::pair<int, int> metric_to_cell(double mx, double my) const;
  std::pair<int, int> world_metric_to_cell(double mx, double my) const;
  std::vector<std::pair<int, int>> ray_trace(int gx, int gy, int px, int py) const;

private:
  size_t width_;
  size_t height_;
  double resolution_; // m / cell
  double origin_x_; // m
  double origin_y_; // m
  std::vector<uint8_t> data_;

};

}  // namespace easynav

#endif  // DRAINY_COMMON__DRAINYMAP_HPP_