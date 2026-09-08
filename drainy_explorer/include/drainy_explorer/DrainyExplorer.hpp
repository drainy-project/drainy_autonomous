#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_system/GoalManagerClient.hpp"

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
    ExplorerState state_ {ExplorerState::IDLE};

    bool initialized_ {false};
    size_t send_retries_ {0};
    const size_t max_retries_ {3};
    uint last_control_type_ {0};

    std::string frame_id_{"map"};
    nav_msgs::msg::Goals goals_;
    GoalManagerClient::SharedPtr gm_client_;
    rclcpp::Time pause_start_time_;
    rclcpp::Duration pause_duration_ = rclcpp::Duration::from_seconds(2.0);
    rclcpp::TimerBase::SharedPtr timer_;

};

} // namespace easynav
