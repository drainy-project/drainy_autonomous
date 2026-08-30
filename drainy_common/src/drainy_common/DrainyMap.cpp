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

#include "drainy_common/DrainyMap.hpp"

namespace easynav
{

DrainyMap::DrainyMap()
: width_(0), height_(0), resolution_(1.0), origin_x_(0.0), origin_y_(0.0), data_()
{}

void
DrainyMap::initialize(
  double m_width, double m_height, double resolution,
  double origin_x, double origin_y)
{
  width_ = m_width / resolution;
  height_ = m_height / resolution;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  data_.assign(width_ * height_, -1);
}

void
DrainyMap::from_laser_scan(const sensor_msgs::msg::LaserScan scan_msg)
{
  data_.assign(width_ * height_, -1);
  for (size_t idx = 0; idx < scan_msg.ranges.size(); ++idx) {
    double distance = scan_msg.ranges[idx];
    
    if (std::isnan(distance) || std::isinf(distance) || distance < scan_msg.range_min || distance > scan_msg.range_max) {
        continue;
    }

    double angle = scan_msg.angle_min + (idx * scan_msg.angle_increment);
    
    double map_origin_x = origin_x_ - resolution_ * (width_ / 2);
    double map_origin_y = origin_y_ - resolution_ * (height_ / 2);

    double wx = distance * std::cos(angle) - map_origin_x;
    double wy = distance * std::sin(angle) - map_origin_y;

    if (wx < 0 || wy < 0){
      std::cerr << "Menor que cero" << std::endl;
    }
    
    size_t gx = static_cast<int>((wx) / resolution_);
    size_t gy = static_cast<int>((wy) / resolution_);
    
    if ((gx < width_) && (gy < height_)) {
        int index = gy * width_ + gx;
        data_[index] = 100; 
    }
  }

}

void
DrainyMap::to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg) const
{
  grid_msg.info.width = static_cast<uint32_t>(width_);
  grid_msg.info.height = static_cast<uint32_t>(height_);
  grid_msg.info.resolution = static_cast<float>(resolution_);

  grid_msg.info.origin.position.x = origin_x_ - resolution_ * (width_ / 2);
  grid_msg.info.origin.position.y = origin_y_ - resolution_ * (height_ / 2);
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.x = 0.0;
  grid_msg.info.origin.orientation.y = 0.0;
  grid_msg.info.origin.orientation.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  if (grid_msg.data.size() != width_ * height_) {
    grid_msg.data.resize(width_ * height_);
  }

  for (size_t idx = 0; idx < data_.size(); ++idx) {
    grid_msg.data[idx] = data_[idx];
  }
}

void
DrainyMap::print(bool view_data) const
{
  std::cerr << "===== DrainyMap Metadata: =====\n";
  std::cerr << "  Size: " << width_ << " x " << height_ << "\n";
  std::cerr << "  Resolution: " << resolution_ << " m/cell\n";
  std::cerr << "  Origin: (" << origin_x_ << ", " << origin_y_ << ")\n";

  if (!view_data) {return;}

  std::cerr << "----- DrainyMap Data: -----\n";
  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      int index = y * width_ + x;
      double mx = origin_x_ + (x + 0.5) * resolution_;
      double my = origin_y_ + (y + 0.5) * resolution_;
      std::cerr << "[" << x << ", " << y << "][" << mx << ", " << my << "] "
                << static_cast<int>(data_[index]) << "\n";
    }
  }
}

// bool
// DrainyMap::save_to_file(const std::string & path) const
// {
//   std::ofstream out(path);
//   if (!out.is_open()) {
//     return false;
//   }

//   out << width_ << " " << height_ << " "
//       << resolution_ << " "
//       << origin_x_ << " " << origin_y_ << "\n";

//   for (size_t i = 0; i < data_.size(); ++i) {
//     out << (data_[i] ? "1" : "0");
//     if (i + 1 < data_.size()) {
//       out << " ";
//     }
//   }

//   out << "\n";
//   return true;
// }

}  // namespace easynav