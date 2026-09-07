#ifndef DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_
#define DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_system/GoalManager.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace easynav
{

class DrainyController : public ControllerMethodBase
{
    public:

    DrainyController();

    ~DrainyController();

    void on_initialize() override;

    void update_rt(NavState & nav_state) override;

    protected:

    geometry_msgs::msg::TwistStamped cmd_vel_;
    sensor_msgs::msg::PointCloud2 cloud_h_msg_;
    sensor_msgs::msg::PointCloud2 cloud_v_msg_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_h_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_v_pub_;

    private:

    double x_gain_{1.0};
    double y_gain_{1.0};
    double z_gain_{1.0};
    double yaw_gain_{0.8};
    double detection_limit_{3.5}; // For realsense
    double safety_radius_{1.0};
    double safety_vertical_{0.3};
    double vel_lineal_max_{0.5};
    double vel_angular_max_{1.0};
    double yaw_limit_{0.1};
    double convergence_limit_{2.0};

};

} // namespace easynav

#endif // DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_