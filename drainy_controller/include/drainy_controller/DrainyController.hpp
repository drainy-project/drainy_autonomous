#ifndef DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_
#define DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_system/GoalManager.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

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

    private:

    double x_gain_{1.0};
    double y_gain_{0.5};
    double z_gain_{0.5};
    double yaw_gain_{0.5};

};

} // namespace easynav

#endif // DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_