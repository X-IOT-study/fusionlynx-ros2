#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>

#include "state_fusion_backend/state_fusion_backend_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<state_fusion_backend::StateFusionBackendNode>();

  // FusionLynx revision 2026-08-21, WenSheng Xu:
  // Start the lifecycle node in the active state so the standalone launch
  // file is immediately usable. External lifecycle managers can still
  // deactivate or clean it through the normal ROS 2 lifecycle services.
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (
    node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
