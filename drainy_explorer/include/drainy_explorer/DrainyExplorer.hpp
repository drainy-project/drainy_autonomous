#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include "easynav_system/GoalManagerClient.hpp"

#include "drainy_common/DrainyMap.hpp"

using namespace std::chrono_literals;

namespace easynav
{

inline geometry_msgs::msg::Quaternion setYaw(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  return tf2::toMsg(q);
}

class DrainyExplorer : public rclcpp::Node
{
    public:
    RCLCPP_SMART_PTR_DEFINITIONS(DrainyExplorer)
    explicit DrainyExplorer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~DrainyExplorer() = default;

    private:
    enum class ExplorerState {IDLE, EXPLORING, FINISHED, ERROR, DO_AT_WAYPOINT};
    void initialize();
    void cycle();
    bool find_goal();
    void set_home();
    ExplorerState state_ {ExplorerState::IDLE};

    bool initialized_ {false};
    bool completed_ {false};
    size_t send_retries_ {0};
    const size_t max_retries_ {3};
    uint last_control_type_ {0};
    double max_long_{20.0};

    DrainyMap drainy_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr incoming_map_sub_;
    rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr incoming_control_sub_;

    std::string frame_id_{"map"};
    nav_msgs::msg::Goals goals_;
    geometry_msgs::msg::PoseStamped current_goal_;
    geometry_msgs::msg::PoseStamped current_home_;
    geometry_msgs::msg::PoseStamped current_pose_;
    GoalManagerClient::SharedPtr gm_client_;
    rclcpp::Time pause_start_time_;
    rclcpp::Duration pause_duration_ = rclcpp::Duration::from_seconds(2.0);
    rclcpp::TimerBase::SharedPtr timer_;

};

} // namespace easynav
