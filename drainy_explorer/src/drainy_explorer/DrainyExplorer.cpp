#include "drainy_explorer/DrainyExplorer.hpp"

namespace easynav
{
  DrainyExplorer::DrainyExplorer(const rclcpp::NodeOptions & options)
  : Node("Exploring_node", options)
  {
    timer_ = create_timer(
          100ms,
          std::bind(&DrainyExplorer::cycle, this));
  }

  void
  DrainyExplorer::initialize()
  {

    goals_.header.frame_id = frame_id_;

      std::vector<double> wp_coord{0.0, 0.0, 0.0};

      geometry_msgs::msg::PoseStamped wp_pose;
      wp_pose.header.frame_id = frame_id_;
      wp_pose.pose.position.x = wp_coord[0];
      wp_pose.pose.position.y = wp_coord[1];
      wp_pose.pose.orientation = setYaw(wp_coord[2]);

      goals_.goals.push_back(wp_pose);
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

          nav_msgs::msg::Goals single_goal;
          single_goal.header = goals_.header;
          ////////////
          geometry_msgs::msg::PoseStamped goal;
          goal.header = single_goal.header;
          goal.pose.position.x = 20.0;
          goal.pose.position.y = 0.0;
          ///////////
          single_goal.goals.push_back(goal); // poner aqui esse valor
  
          // while (gm_client_->get_state() != GoalManagerClient::State::IDLE)
          // {
          //   gm_client_->reset(); // Ensure the client is idle before sending new goals
          // }

          gm_client_->send_goals(single_goal);
          RCLCPP_INFO(get_logger(), "Goals sent");
          state_ = ExplorerState::EXPLORING;
        }
        break;

      case ExplorerState::EXPLORING:
        {
          int current_goal_index_ = 0; //cambiar a heuristica
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
              RCLCPP_INFO(get_logger(), "Waiting time started at waypoint %u",
              current_goal_index_ + 1);

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
        // DONE: 
        {
          int current_goal_index_ = 0; //cambiar a heuristica
          if (now() - pause_start_time_ >= pause_duration_) {
            RCLCPP_INFO(get_logger(), "Waiting time ended at waypoint %u", current_goal_index_ + 1);

            // advance to next waypoint
            
            if (current_goal_index_ < goals_.goals.size()) {
              RCLCPP_INFO(get_logger(), "Navigating to waypoint %u", current_goal_index_ + 1);
              gm_client_->reset();
              state_ = ExplorerState::IDLE;
            } else {
              RCLCPP_INFO(get_logger(), "All waypoints completed");
              state_ = ExplorerState::FINISHED;
            }
          }
          break;
        }


      case ExplorerState::FINISHED:
        RCLCPP_INFO(get_logger(), "Reset navigation");
        //current_goal_index_ = 0;
        state_ = ExplorerState::IDLE;
        if (gm_client_->get_state() != GoalManagerClient::State::IDLE) {
          gm_client_->reset();
        }
        break;

      case ExplorerState::ERROR:
        break;
    }
  }

} // namespace easynav