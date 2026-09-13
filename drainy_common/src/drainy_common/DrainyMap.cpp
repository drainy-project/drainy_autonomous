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

std::vector<std::pair<int, int>> mask_neighbors = {
  {1, 1}, {1, 0}, {1, -1},
  {0, 1}, {0, 0}, {0, -1},
  {-1, 1}, {-1, 0}, {-1, -1}
};

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

std::pair<double, double>
DrainyMap::cell_to_metric(int x, int y) const
{
  double mx = origin_x_ + (static_cast<double>(x) + 0.1) * resolution_;
  double my = origin_y_ + (static_cast<double>(y) + 0.1) * resolution_;
  return {mx, my};
}

std::pair<int, int>
DrainyMap::metric_to_cell(double mx, double my) const
{
    double wx = mx - origin_x_;
    double wy = my - origin_x_;

    int x = static_cast<int>((wx) / resolution_);
    int y = static_cast<int>((wy) / resolution_);

  return {x, y};
}

std::pair<int, int>
DrainyMap::world_metric_to_cell(double mx, double my) const
{
    double map_origin_x = origin_x_ - resolution_ * (width_ / 2);
    double map_origin_y = origin_y_ - resolution_ * (height_ / 2);

    double wx = mx - map_origin_x;
    double wy = my - map_origin_y;

    int x = static_cast<int>((wx) / resolution_);
    int y = static_cast<int>((wy) / resolution_);

  return {x, y};
}

void
DrainyMap::from_pc2(const sensor_msgs::msg::PointCloud2 msg, 
  geometry_msgs::msg::Pose pose,
  double confidence_lenght)
{
  //data_.assign(width_ * height_, -1);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x"),
      iter_y(msg, "y"), iter_z(msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      continue;
    }

    // double map_origin_x = origin_x_ - resolution_ * (width_ / 2);
    // double map_origin_y = origin_y_ - resolution_ * (height_ / 2);

    // double wx = *iter_x - map_origin_x;
    // double wy = *iter_y - map_origin_y;

    // size_t gx = static_cast<int>((wx) / resolution_);
    // size_t gy = static_cast<int>((wy) / resolution_);

    auto [gx, gy] = world_metric_to_cell(*iter_x, *iter_y);

    if ((gx < width_) && (gy < height_)) {
        int index = gy * width_ + gx;
        if (data_[index] != 0){
          data_[index] = 100;
        }
        
    }
  }
  auto [px, py] = world_metric_to_cell(pose.position.x, pose.position.y);
  for (int i = 0; i < static_cast<int>(confidence_lenght / resolution_); ++i){
    for (auto [dx, dy] : mask_neighbors){
      int x = px + dx * i;
      int y = py + dy * i;
      int index = y * width_ + x;
      data_[index] = 0;
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
DrainyMap::from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid_msg)
{
  initialize(
    grid_msg.info.width,
    grid_msg.info.height,
    grid_msg.info.resolution,
    grid_msg.info.origin.position.x + grid_msg.info.resolution * (grid_msg.info.width / 2),
    grid_msg.info.origin.position.y + grid_msg.info.resolution * (grid_msg.info.height / 2));

  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      size_t idx = y * width_ + x;
      int8_t val = grid_msg.data[idx];
      data_[idx] = val;
    }
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