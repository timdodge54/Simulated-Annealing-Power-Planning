#ifndef PowerNode_hpp
#define PowerNode_hpp
#include "cc_msgs/msg/battery_level.hpp"
#include "cc_msgs/msg/charge_level.hpp"
#include "cc_msgs/srv/load_profile.hpp"
#include "power_energy_propogation/data_struct.hpp"
#include "power_energy_propogation/planner.hpp"
#include "power_energy_propogation/power_solution.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cc_msgs/msg/detail/battery_level__struct.hpp>
#include <cc_msgs/srv/detail/load_profile__struct.hpp>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
using namespace std::chrono_literals;
namespace ConnectedComm {
/**
 * @brief  Class that provides a ros interface for the battery aggregate.
 * @details This class provides a ros interface for the battery aggregate. It
 *takes an exteranl load profile and parameters for the simulation and creates
 *a battery aggregate. It then publishes the charge levels of the batteries.
 */
class PowerNode : public rclcpp::Node {
public:
  std::mutex
      mutex_lock;    // mutex lock for thread safety when publishing charge info
  int time_horizon;  // Number of timesteps in the simulation
  int num_batteries; // Number of batteries in the simulation
  int num_add_events;    // Number of random events to be generated
  int num_remove_events; // number of events to be randomly removed
  double battery_rate;   // Rate at which the battery can be charged/discharged
  double battery_capacity; // Capacity of the battery
  double time_step;        // Time step of the simulation
  double temp;
  double temp_cooling_rate;
  Planner planner;                 // Simulated anneahling object
  PowerSolution battery_aggregate; // Battery aggregate object
  rclcpp::Publisher<cc_msgs::msg::ChargeLevel>::SharedPtr
      charge_levels_pub; // Publisher for charge levels
  rclcpp::Client<cc_msgs::srv::LoadProfile>::SharedPtr
      load_profile_client;            // Client for load profile service
  std::vector<float> time;            // Vector of times
  std::vector<float> post_sol_time;   // Vector of times for post solution
  std::vector<double> batt_cap;       // Vector of battery charge levels
  std::vector<double> batt_min;       // Vector of battery charge levels
  std::vector<double> batt_rate;      // Vector of battery charge levels
  rclcpp::TimerBase::SharedPtr timer; // Timer for publishing charge levels
  rclcpp::TimerBase::SharedPtr
      timer2; // Timer for publishing post solution charge levels
  std::default_random_engine re; // Random engine for generating random events
  std::vector<double> load_profile;
  std::string planner_type;
  double soc_start;
  double temp_min;
  double fitness_scalar;
  int planner_number;

  /**
   * @brief Decalres all ros parameters needed for the simulation
   **/
  inline void declare_parameters() {
    std::vector<double> battery_capacaties;
    battery_capacaties.push_back(100.);
    std::vector<double> battery_rates;
    battery_rates.push_back(300.);
    std::vector<double> battery_minimum;
    battery_minimum.push_back(100.);
    this->declare_parameter("time_horizon", 1400);
    this->declare_parameter("time_step", 1.);
    this->declare_parameter("battery_rate", 300.);
    this->declare_parameter("battery_capacity", 100.);
    this->declare_parameter("num_batteries", 1);
    this->declare_parameter("num_add_events", 1000);
    this->declare_parameter("temp", 1.0);
    this->declare_parameter("planner_type", "GREEDY");
    this->declare_parameter("temp_cooling_rate", 0.99);
    this->declare_parameter("battery_capacaties", battery_capacaties);
    this->declare_parameter("battery_rates", battery_rates);
    this->declare_parameter("battery_minimums", battery_minimum);
    this->declare_parameter("soc_start", 150.0);
    this->declare_parameter("temp_min", 2.0);
    this->declare_parameter("fitness_scalar", 200.0);
    this->declare_parameter("planner_number", 0);
  }
  /**
   * @brief Gets all ros parameters needed for the simulation
   **/
  inline void get_parameters() {
    time_horizon = this->get_parameter("time_horizon").as_int();
    battery_rate = this->get_parameter("battery_rate").as_double();
    battery_capacity = this->get_parameter("battery_capacity").as_double();
    time_step = this->get_parameter("time_step").as_double();
    num_batteries = this->get_parameter("num_batteries").as_int();
    num_add_events = this->get_parameter("num_add_events").as_int();
    temp_cooling_rate = this->get_parameter("temp_cooling_rate").as_double();
    temp = this->get_parameter("temp").as_double();
    batt_cap = this->get_parameter("battery_capacaties").as_double_array();
    batt_rate = this->get_parameter("battery_rates").as_double_array();
    batt_min = this->get_parameter("battery_minimums").as_double_array();
    planner_type = this->get_parameter("planner_type").as_string();
    soc_start = this->get_parameter("soc_start").as_double();
    temp_min = this->get_parameter("temp_min").as_double();
    fitness_scalar = this->get_parameter("fitness_scalar").as_double();
    planner_number = this->get_parameter("planner_number").as_int();
    if (!(planner_type.compare("GREEDY") != 0 or
          planner_type.compare("SA") != 0)) {
      throw std::runtime_error("Invalid planner type");
    } else {
    }
  }
  /**
   * @brief Construct a new Power Node object
   */
  PowerNode();

  /**
   * @brief The callback that will regularly publish the charge levels.
   */
  void publish_charge_levels();
};
} // namespace ConnectedComm
#endif // PowerNode_hpp
