#ifndef DRAINY__PLANNER__DRAINYLOCALIZER_HPP_
#define DRAINY__PLANNER__DRAINYLOCALIZER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace easynav
{

class DrainyPlanner : public PlannerMethodBase
{
    public:

    DrainyPlanner();

    ~DrainyPlanner();

    void on_initialize() override;

    void update(NavState & nav_state) override;

    protected:

    double error_{0.0};
    double min_error_{0.1};
    bool error_updated_{true};
    nav_msgs::msg::Path current_path_;  
    geometry_msgs::msg::Pose current_goal_;  
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_pub_;

    std::vector<geometry_msgs::msg::Pose> get_poses(
    const std::vector<std::shared_ptr<easynav::PointPerception>> & perceptions,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

};

} // namespace easynav

#endif // DRAINY__PLANNER__DRAINYLOCALIZER_HPP_