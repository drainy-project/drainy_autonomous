#ifndef DRAINY__LOCALIZER__DRAINYLOCALIZER_HPP_
#define DRAINY__LOCALIZER__DRAINYLOCALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace easynav
{

class DrainyLocalizer : public LocalizerMethodBase
{
    public:

    DrainyLocalizer();

    ~DrainyLocalizer();

    void on_initialize() override;

    void update_rt(NavState & nav_state) override;

    void update(NavState & nav_state) override;

    tf2::Transform get_pose(void);

    protected:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void publish_odom_TF(const tf2::Transform & map2odom);

    void publish_bf_TF(const tf2::Transform & map2bf);

    void init_odom(void);

    void printTransform(const tf2::Transform & tf);

    void set_init_pose(double x, double y, double z, double yaw);

    nav_msgs::msg::Odometry get_odom(void);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    nav_msgs::msg::Path trajectory_;  

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::Odometry::SharedPtr odom_msg_{nullptr};

    geometry_msgs::msg::Pose::SharedPtr init_pose_{nullptr};

    tf2::Transform odom_tf_{tf2::Transform::getIdentity()};

    rclcpp::Time last_input_time_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool initialized_odom_{false};

};

} // namespace easynav

#endif // DRAINY__LOCALIZER__DRAINYLOCALIZER_HPP_