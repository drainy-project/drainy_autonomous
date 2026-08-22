#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace easynav
{

class DrainyExplorer : public rclcpp::Node
{
    public:
    DrainyExplorer();
    ~DrainyExplorer();

    private:
    void timer_callback(void);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;


};

} // namespace easynav
