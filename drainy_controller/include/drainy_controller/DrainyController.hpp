#ifndef DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_
#define DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/ControllerMethodBase.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

class DrainyController : public ControllerMethodBase
{
    public:

    DrainyController();

    ~DrainyController();

    void on_initialize() override;

    void update_rt(NavState & nav_state) override;

};

} // namespace easynav

#endif // DRAINY__CONTROLLER__DRAINYLOCALIZER_HPP_