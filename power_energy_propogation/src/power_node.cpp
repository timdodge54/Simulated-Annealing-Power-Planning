#include "power_energy_propogation/power_node.hpp"

namespace ConnectedComm {
/**
 * @brief Construct a new Power Node object
 */
PowerNode::PowerNode() : Node("list_tester") {
  RCLCPP_INFO(this->get_logger(), "PowerNode node has been created.");
  // Declare and get parameters
  this->declare_parameters();
  this->get_parameters();
  // Create publisher and client
  this->charge_levels_pub =
      this->create_publisher<cc_msgs::msg::ChargeLevel>("charge_level", 10);
  load_profile_client =
      this->create_client<cc_msgs::srv::LoadProfile>("load_profile");
  // Create the request that will define the uncontrolled load profile
  auto req = std::make_shared<cc_msgs::srv::LoadProfile::Request>();
  req->timestep = this->time_step;
  req->nsteps = this->time_horizon;
  // Wait for the uncontrolled load profile service to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for service");
  while (!load_profile_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "Service available");
  auto result = this->load_profile_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Service call successful");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
  }
  RCLCPP_INFO(this->get_logger(), "Service call successful");
  // Get the resulting load profile
  load_profile = result.get()->load_profile;

  RCLCPP_INFO(this->get_logger(), "Load profile size: %ld",
              load_profile.size());
  if (planner_type.compare("GREEDY") == 0) {
    this->planner =
        Planner(time_horizon, num_batteries, time_step, load_profile, batt_cap,
                batt_rate, batt_min, temp, num_add_events, planner_type::GREEDY,
                temp_cooling_rate, soc_start, temp_min, fitness_scalar);
  } else {
    this->planner =
        Planner(time_horizon, num_batteries, time_step, load_profile, batt_cap,
                batt_rate, batt_min, temp, num_add_events, planner_type::SA,
                temp_cooling_rate, soc_start, temp_min, fitness_scalar);
  }
  RCLCPP_INFO(this->get_logger(), "Publishing Solution for planner");
  // create timer for publishing charge levels
  this->timer2 = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&PowerNode::publish_charge_levels, this));
  RCLCPP_INFO(this->get_logger(), "Publishing Solution Initialized");
}
/**
 * @brief The callback that will regularly publish the charge levels.
 */
void PowerNode::publish_charge_levels() {
  // Create the message to publish
  cc_msgs::msg::ChargeLevel charge_level_msg;
  // Get the max and min time slot idx
  const int max_idx = this->planner.best_sol.get_max_time_slot_idx();
  const int min_idx = this->planner.best_sol.get_min_time_slot_idx();
  // Populate the message with max and min
  charge_level_msg.max_time_idx = max_idx;
  charge_level_msg.min_time_idx = min_idx;
  // Create Battery Level message for total power constumption
  cc_msgs::msg::BatteryLevel total_level;
  RCLCPP_DEBUG(this->get_logger(), "Max idx: %d, Min idx: %d", max_idx,
               min_idx);
  // Create vector of battery levels for each battery
  std::vector<cc_msgs::msg::BatteryLevel> battery_levels;
  const uint num_batteries =
      this->planner.best_sol.battery_id_to_energy_time_slots.size();
  for (uint i = 0; i < num_batteries; ++i) {
    battery_levels.push_back(cc_msgs::msg::BatteryLevel());
  }
  // get the number of time slots and number of batteries
  const uint size_of_time_slots =
      this->planner.best_sol.battery_id_to_energy_time_slots[0].size();
  for (uint i = 0; i < size_of_time_slots; ++i) {
    // get the extenal power draw for the time solot
    const EnergyTimeSlot &external_slot =
        this->planner.best_sol.external_source[i];
    // initialize the total power for the time slot with the external power
    double total_power_for_slot = external_slot.power;
    // add the time idx to the message
    charge_level_msg.time_idxs.push_back(external_slot.time_idx);
    for (uint j = 0; j < num_batteries; ++j) {
      // get the power and energy for timeslot of a specific battery
      const EnergyTimeSlot &time_slot =
          this->planner.best_sol.battery_id_to_energy_time_slots[j][i];
      total_power_for_slot += time_slot.power;
      // push the power and energy for the time slot
      battery_levels[j].power_level.push_back(time_slot.power);
      battery_levels[j].charge_level.push_back(time_slot.energy);
    }
    // push the total power for the time slot
    charge_level_msg.resulting_signal.push_back(total_power_for_slot);
  }
  // set the batteries field and external signal
  charge_level_msg.charge_levels = battery_levels;
  charge_level_msg.external_signal = load_profile;
  charge_level_msg.primary_fitness_history =
      this->planner.get_primary_history();
  charge_level_msg.secondary_fitness_history =
      this->planner.get_secondary_history();
  charge_level_msg.tertiary_fitness_history =
      this->planner.get_tertiary_history();
  charge_level_msg.planner_num = planner_number;
  charge_levels_pub->publish(charge_level_msg);
}
} // namespace ConnectedComm
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto power_node = std::make_shared<ConnectedComm::PowerNode>();
  rclcpp::spin(power_node);
  rclcpp::shutdown();
  return 0;
}
