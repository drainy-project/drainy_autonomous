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
  int width, int height, double resolution,
  double origin_x, double origin_y)
{
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  data_.assign(width * height, -1);
}

void
DrainyMap::to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg,
  sensor_msgs::msg::LaserScan scan_msg) const
{
  grid_msg.info.width = static_cast<uint32_t>(width_);
  grid_msg.info.height = static_cast<uint32_t>(height_);
  grid_msg.info.resolution = static_cast<float>(resolution_);
  grid_msg.header = scan_msg.header; 

  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.x = 0.0;
  grid_msg.info.origin.orientation.y = 0.0;
  grid_msg.info.origin.orientation.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  if (grid_msg.data.size() != width_ * height_) {
    grid_msg.data.resize(width_ * height_);
  }
  grid_msg.data.assign(width_ * height_, -1);
  for (size_t idx = 0; idx < grid_msg.data.size(); ++idx) {
    double dist = scan_msg.ranges[idx];
    
    // Filtrar rangos válidos e ignorar lecturas erróneas (NaN / Inf)
    if (std::isnan(dist) || std::isinf(dist) || dist < scan_msg.range_min || dist > scan_msg.range_max) {
        continue;
    }

    // Calcular ángulo absoluto del rayo
    double angle = scan_msg.angle_min + (idx * scan_msg.angle_increment);
    
    // Coordenadas del obstáculo en el mundo real
    double wx = origin_x_ + dist * std::cos(angle);
    double wy = origin_y_ + dist * std::sin(angle);
    
    // Convertir coordenadas del mundo a índices de la matriz
    int gx = static_cast<int>((wx) / resolution_);
    int gy = static_cast<int>((wy) / resolution_);
    
    // Verificar límites de la matriz y marcar como ocupado (100)
    if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
        int index = gy * height_ + gx;
        grid_msg.data[index] = 100; // 100 = Celda Ocupada
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