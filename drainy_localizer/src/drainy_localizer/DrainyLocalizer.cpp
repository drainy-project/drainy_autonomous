#include "drainy_localizer/DrainyLocalizer.hpp"

namespace easynav
{
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    DrainyLocalizer::DrainyLocalizer()
    {
        NavState::register_printer<nav_msgs::msg::Odometry>(
            [](const nav_msgs::msg::Odometry & odom) {
            std::ostringstream ret;
            double x = odom.pose.pose.position.x;
            double y = odom.pose.pose.position.y;
            double z = odom.pose.pose.position.z;

            tf2::Quaternion q(
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            ret << "{" << rclcpp::Time(odom.header.stamp).seconds() << "} Odometry with pose: (x: " <<
                x << ", y: " << y << ", z: " << z << ", yaw: " << yaw << ")";
            return ret.str();
            });
    }

    DrainyLocalizer::~DrainyLocalizer(){}

    void DrainyLocalizer::on_initialize()
    {

        auto node = get_node();
        const std::string & plugin_name = this->get_plugin_name();

        double x_init, y_init, z_init, yaw_init;

        node->declare_parameter<double>(plugin_name + ".initial_pose.x", 0.0);
        node->declare_parameter<double>(plugin_name + ".initial_pose.y", 0.0);
        node->declare_parameter<double>(plugin_name + ".initial_pose.z", 0.0);
        node->declare_parameter<double>(plugin_name + ".initial_pose.yaw", 0.0);

        node->get_parameter<double>(plugin_name + ".initial_pose.x", x_init);
        node->get_parameter<double>(plugin_name + ".initial_pose.y", y_init);
        node->get_parameter<double>(plugin_name + ".initial_pose.z", z_init);
        node->get_parameter<double>(plugin_name + ".initial_pose.yaw", yaw_init);

        RCLCPP_INFO(node->get_logger(), "%s plugin has been initialized", plugin_name.c_str());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/genz/odometry", qos,
            std::bind(&DrainyLocalizer::odom_callback, this, std::placeholders::_1));

        path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            node->get_name() + std::string("/") + plugin_name + "/trajectory", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
        
        init_pose_ = std::make_shared<geometry_msgs::msg::Pose>();

        set_init_pose(x_init, y_init, z_init, yaw_init);
    }

    void DrainyLocalizer::set_init_pose(double x, double y, double z, double yaw)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Set init_position");

        init_pose_->position.x = x;
        init_pose_->position.y = y;
        init_pose_->position.z = z;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        init_pose_->orientation.x = q.x();
        init_pose_->orientation.y = q.y();
        init_pose_->orientation.z = q.z();
        init_pose_->orientation.w = q.w();
    }

    void DrainyLocalizer::printTransform(const tf2::Transform & tf)
    {
        const tf2::Vector3 & origin = tf.getOrigin();
        const tf2::Quaternion & rot = tf.getRotation();

        std::cerr << "Translation: ["
                    << origin.x() << ",\n "
                    << origin.y() << ",\n "
                    << origin.z() << "]\n";

        std::cerr << "Rotation (quaternion): [\n"
                    << rot.x() << ",\n"
                    << rot.y() << ",\n"
                    << rot.z() << ",\n"
                    << rot.w() << "]\n";
    }

    void
    DrainyLocalizer::init_odom()
    {
        // To DO - Change for parameters
        tf2::Vector3 pose(init_pose_->position.x, 
                         init_pose_->position.y, 
                         init_pose_->position.z);

        double roll, pitch, yaw;
        tf2::Quaternion q_odom(init_pose_->orientation.x,
                              init_pose_->orientation.y,
                              init_pose_->orientation.z,
                              init_pose_->orientation.w);
        tf2::Matrix3x3(q_odom).getRPY(roll, pitch, yaw);
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        tf2::Transform est;
        est.setOrigin(pose);
        est.setRotation(q);
        publish_odom_TF(est);

        initialized_odom_ = true;
    }

    void
    DrainyLocalizer::publish_odom_TF(const tf2::Transform & map2odom)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = last_input_time_;
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        tf_msg.header.frame_id = tf_info.map_frame;
        tf_msg.child_frame_id = tf_info.odom_frame;
        tf_msg.transform = tf2::toMsg(map2odom);
        RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void
    DrainyLocalizer::publish_bf_TF(const tf2::Transform & map2bf)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = last_input_time_;
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        tf_msg.header.frame_id = tf_info.odom_frame;
        tf_msg.child_frame_id = tf_info.robot_frame;
        tf_msg.transform = tf2::toMsg(map2bf);
        RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
        tf_broadcaster_->sendTransform(tf_msg);
    }


    void DrainyLocalizer::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg_ = msg;
        tf2::fromMsg(odom_msg_->pose.pose, odom_tf_);
        last_input_time_ = odom_msg_->header.stamp;
        tf2::Transform map2bf = get_pose();
        tf2::Transform map2odom = map2bf * odom_tf_.inverse(); // Should be zero
        
        publish_bf_TF(map2bf);
        publish_odom_TF(map2odom);

        // RCLCPP_INFO(get_node()->get_logger(), "map2odom: ");
        // printTransform(map2odom);

        // RCLCPP_INFO(get_node()->get_logger(), "map2bf: ");
        // printTransform(map2bf);
    }

    tf2::Transform
    DrainyLocalizer::get_pose()
    {
        tf2::Vector3 pose(odom_msg_->pose.pose.position.x, 
                         odom_msg_->pose.pose.position.y, 
                         odom_msg_->pose.pose.position.z);

        double roll, pitch, yaw;
        tf2::Quaternion q_odom(odom_msg_->pose.pose.orientation.x,
                              odom_msg_->pose.pose.orientation.y,
                              odom_msg_->pose.pose.orientation.z,
                              odom_msg_->pose.pose.orientation.w);
        tf2::Matrix3x3(q_odom).getRPY(roll, pitch, yaw);
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        tf2::Transform est;
        est.setOrigin(pose);
        est.setRotation(q);
        return est;
    }

    nav_msgs::msg::Odometry 
    DrainyLocalizer::get_odom()
    {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = last_input_time_;
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        odom_msg.header.frame_id = tf_info.map_frame;
        odom_msg.child_frame_id = tf_info.robot_frame;

        odom_msg.pose.pose.position.x = odom_tf_.getOrigin().x();
        odom_msg.pose.pose.position.y = odom_tf_.getOrigin().y();
        odom_msg.pose.pose.position.z = odom_tf_.getOrigin().z();
        odom_msg.pose.pose.orientation = tf2::toMsg(odom_tf_.getRotation());

        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        return odom_msg;
    }

    void DrainyLocalizer::update_rt([[maybe_unused]] NavState & nav_state)
    {
        if(!initialized_odom_)
        {
            init_odom();
            return;
        }
        nav_state.set("robot_pose", get_odom());
    }

    void DrainyLocalizer::update([[maybe_unused]] NavState & nav_state)
    {
        if(!initialized_odom_)
        {
            init_odom();
            return;
        }
        nav_msgs::msg::Odometry odom = get_odom();
        nav_state.set("robot_pose", odom);
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        trajectory_.header.stamp = get_node()->now();
        trajectory_.header.frame_id = tf_info.map_frame;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = trajectory_.header;
        pose_stamped.pose = odom.pose.pose;
        trajectory_.poses.push_back(pose_stamped);
        path_pub_->publish(trajectory_); 
    }
} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyLocalizer, easynav::LocalizerMethodBase)