#include "drainy_explorer/DrainyExplorer.hpp"
#include <array>

namespace easynav
{
  DrainyExplorer::DrainyExplorer(const rclcpp::NodeOptions & options)
  : Node("explorer_node", options)
  {
    timer_ = create_timer(
          100ms,
          std::bind(&DrainyExplorer::cycle, this));

    incoming_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/maps_manager_node/drainy/map",
          rclcpp::QoS(100),
          [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr raw_msg) {
            drainy_map_.from_occupancy_grid(*raw_msg);
            }
          );
           
    incoming_control_sub_ = create_subscription<easynav_interfaces::msg::NavigationControl>(
        "/easynav_control",
        rclcpp::QoS(100),
        [&](easynav_interfaces::msg::NavigationControl raw_msg) {
          current_pose_.pose = raw_msg.current_pose.pose;
          }
        );    

  }

  void
  DrainyExplorer::set_home()
  {
    current_home_.header.frame_id = frame_id_;
    current_home_.pose.position.x = current_pose_.pose.position.x;
    current_home_.pose.position.y = current_pose_.pose.position.y;
    current_home_.pose.orientation = current_pose_.pose.orientation;
    RCLCPP_INFO(get_logger(), "Home position was defined at X:= %lf Y:= %lf", 
          current_home_.pose.position.x,
          current_home_.pose.position.y);
  }

  bool
  DrainyExplorer::search_goal_in_direction(const SearchDirection& direction, std::vector<double>& goal)
  {
    auto width = drainy_map_.width();
    auto [px, py] = drainy_map_.world_metric_to_cell( 
            current_pose_.pose.position.x, current_pose_.pose.position.y);

    for(int i = static_cast<int>(max_long_ / drainy_map_.resolution()); i > 0 ; --i)
    {
      int x = px + (direction.dx * i);
      int y = py + (direction.dy * i);
      int index = y * width + x;
      
      if(index < 0) {
        break;
      }
      if (drainy_map_.get_data(index) == 0)
      {
        auto [gx, gy] = drainy_map_.world_cell_to_metric(x, y);
        goal[0] = gx;
        goal[1] = gy;
        goal[2] = tf2::getYaw(current_pose_.pose.orientation) + direction.yaw_offset;
        RCLCPP_INFO(get_logger(), "%s case", direction.name.c_str());
        return true;
      }
    }
    return false;
  }

  bool
  DrainyExplorer::find_goal()
  {
    auto width = drainy_map_.width();
    if (width < 1)
    {
      RCLCPP_INFO(get_logger(), "The map is not available");
      return false;
    }
    
    std::vector<double> goal{0.0, 0.0, 0.0};
    
    std::array<SearchDirection, 4> directions = {{
      {1, 0, 0.0, "Forward"},
      {0, 1, M_PI / 2.0, "Left"},
      {0, -1, -M_PI / 2.0, "Right"},
      {-1, 0, M_PI, "Backward"}
    }};

    for (const auto& direction : directions)
    {
      if (search_goal_in_direction(direction, goal))
      {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = frame_id_;
        goal_pose.pose.position.x = goal[0];
        goal_pose.pose.position.y = goal[1];
        goal_pose.pose.orientation = setYaw(goal[2]);
        goals_.goals.push_back(goal_pose);
        gm_client_->send_goals(goals_);
        RCLCPP_INFO(get_logger(), "Goal sent at X:= %lf Y:= %lf", goal[0], goal[1]);
        return true;
      }
    }

    RCLCPP_INFO(get_logger(), "No Goals availables");
    return false;
  }

  void
  DrainyExplorer::initialize()
  {

    goals_.header.frame_id = frame_id_;

    set_home();

  }

  void
  DrainyExplorer::cycle()
  {
    switch (state_) {
      case ExplorerState::IDLE:
        {
          if (!initialized_) {
            RCLCPP_INFO(get_logger(), "Initializing exploring");
            gm_client_ = GoalManagerClient::make_shared(shared_from_this());
            initialize();
            initialized_ = true;
          }

          // TO DO service in IDLE state to decide

          if (find_goal() && !completed_){
            RCLCPP_INFO(get_logger(), "Exploration has been activated");
            state_ = ExplorerState::EXPLORING;
          }
          
        }
        break;

      case ExplorerState::EXPLORING:
        {
          auto nav_state = gm_client_->get_state();
          switch (nav_state) {
            case GoalManagerClient::State::SENT_GOAL:
              last_control_type_ = gm_client_->get_last_control().type;

              if (last_control_type_ == easynav_interfaces::msg::NavigationControl::REQUEST) {
                if (send_retries_ < max_retries_) {
                  send_retries_++;
                  RCLCPP_INFO(get_logger(), "Waiting for ACCEPT... attempt %zu/%zu", send_retries_,
                  max_retries_);
                } else {
                  RCLCPP_WARN(get_logger(), "No ACCEPT received after %zu attempts, resending goal",
                  max_retries_);
                  send_retries_ = 0;
                  state_ = ExplorerState::IDLE;
                }
              } else if (last_control_type_ == easynav_interfaces::msg::NavigationControl::ACCEPT) {
                send_retries_ = 0;
              }
              break;

            case GoalManagerClient::State::NAVIGATION_REJECTED:
            case GoalManagerClient::State::NAVIGATION_FAILED:
            case GoalManagerClient::State::NAVIGATION_CANCELLED:
            case GoalManagerClient::State::ERROR:
              RCLCPP_ERROR(get_logger(), "Navigation finished with error %s",
                      gm_client_->get_result().status_message.c_str());
              state_ = ExplorerState::ERROR;
              break;
            case GoalManagerClient::State::NAVIGATION_FINISHED:
              RCLCPP_INFO(get_logger(), "Navigation succesfully finished with message %s",
                      gm_client_->get_result().status_message.c_str());

              pause_start_time_ = now();
              state_ = ExplorerState::DO_AT_WAYPOINT;
              break;
            case GoalManagerClient::State::ACCEPTED_AND_NAVIGATING:
              break;
            default:
              break;
          }
        }
        break;

      case ExplorerState::DO_AT_WAYPOINT:
        gm_client_->reset(); 
        {  
          if (find_goal()){
            RCLCPP_INFO(get_logger(), "Goal sent");
            state_ = ExplorerState::EXPLORING;
          } else {
            RCLCPP_INFO(get_logger(), "All map completed");
            state_ = ExplorerState::FINISHED;
          }
          break;
        }

      case ExplorerState::FINISHED:
        RCLCPP_INFO(get_logger(), "Exploration has been finished");
        state_ = ExplorerState::IDLE;
        completed_ = true;
        if (gm_client_->get_state() != GoalManagerClient::State::IDLE) {
          gm_client_->reset();
        }
        break;

      case ExplorerState::ERROR:
        break;
    }
  }

} // namespace easynav