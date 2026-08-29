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

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace easynav
{

class DrainyMap
{
public:

  DrainyMap();

  void initialize(
    int width, int height, double resolution, double origin_x, double origin_y);

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
  void to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg,
    sensor_msgs::msg::LaserScan scan_msg) const;

  /**
  * @brief Saves the map to a file, including metadata and cell data.
  * @param path Path to the output file.
  * @return true if the file was written successfully, false otherwise.
  */
  //bool save_to_file(const std::string & path) const;

private:
  size_t width_;
  size_t height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<uint8_t> data_;
};

}  // namespace easynav

#endif  // DRAINY_COMMON__DRAINYMAP_HPP_