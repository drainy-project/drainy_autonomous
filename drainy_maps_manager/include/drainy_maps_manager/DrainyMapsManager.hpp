#ifndef DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_
#define DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace easynav
{
    class DrainyMapsManager : public MapsManagerBase
    {
        public:
        DrainyMapsManager();
        ~DrainyMapsManager();

        virtual void on_initialize() override;

        virtual void update(NavState & nav_state) override;

        protected:
        std::string map_path_ {"/tmp/drainy_maps/"};
        std::string map_topic_ {"/genz/local_map"};

        private:
        // mapa crudo

        // mapa modificado

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

    }; 

} // namesapce easynav

#endif // DRAINY__MAPSMANAGER__DRAINYMAPSMANAGER_HPP_