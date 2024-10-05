#ifndef PowerSolution_HPP
#define PowerSolution_HPP
#include "power_energy_propogation/data_struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <rclcpp/logger.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace ConnectedComm {
/**
 * @class BatteryAggregation
 * @brief Class to hold the battery aggregation information in a binned format.
 * The class tracks charge accumulation and average power demand in each time
 * bin. It also holds a map of all events added to the list.
 * @attr map_of_events (std::map<int, Event>): Map of all events
 * @attr sorted_time_slot_idxs (std::vector<int>): Vector of sorted time slot
 * only by their index within the chronological ordered time slot bins.
 * @attr time_slots (std::vector<TimeSlot>): Vector of all time slots in
 * chronilogical order.
 * @attr logger (rclcpp::Logger): Logger for the class.
 * @attr time_step (float): Time step between each time slots.
 **/
class PowerSolution {
private:
  std::map<int, Event> map_of_events; // a map from event id to event
  rclcpp::Logger logger = rclcpp::get_logger("battery_agg"); // the ros logger
  std::vector<Battery> batteries;
  float time_step;                 // the time step between two time slots
  unsigned long int failed_events; // the number of failed events
  int num_time_slots;              // the number of time slots
  int num_batteries;               // the number of batteries
  double soc_start;                // The starting soc for the solution
  double total_max_power;          // the maximum power
  double total_min_power;
  int max_idx;    // the maximum power index
  double avg_soc; // the average soc

public:
  std::map<int, std::vector<EnergyTimeSlot>> battery_id_to_energy_time_slots;
  std::vector<EnergyTimeSlot> external_source;
  PowerSolution() = default;
  /**
   * @brief Constructor for the BatteryAggregation class.
   * @param num_time_slots (int): Number of time slots.
   * @param num_batteries (int): Number of batteries.
   * @param time_step (int): The delta t between the begging of time slots
   * @param load_profile (std::vector<double>): The load profile of the
   *uncontrolled load
   * @param soc_min (double): Min soc allowed by the battery
   * @param soc_max (double): Max soc allowed by the battery
   **/
  PowerSolution(const int num_time_slots, const int num_batteries,
                const float time_step, const std::vector<double> load_profile,
                const std::vector<double> batt_cap,
                const std::vector<double> batt_rate,
                const std::vector<double> batt_min,
                const double soc_start = 150.0);
  /*
   * @brief This function adds an event of interaction with a given battery.
   * An event is a time interval where the battery is being used as well as
   * a givens power demand of an event which is the product of the activity
   *value (which has a value between -1 and 1) and the battery rate.
   * @param start_time (double): Start time of the event.
   * @param end_time (double): End time of the event.
   * @param activity (double): Activity value of the event (-1 to 1 inclusive).
   * @param rate (double): Max Rate at which the the battery can charge and
   * discharge.
   **/
  int add_event(const double start_time, const double end_time,
                const double activity, const double rate, const int battery_id);
  /**
   * @brief Searches the solution from the start time to the end of the solution
   * to check if the soc or power constraints will be vioalted.
   **/
  std::tuple<double, double>
  get_bounds_of_problem(const int id, const int start_idx, const int end_idx);
  /**
   * @brief This function prints the current state of the events
   **/
  inline void print_events() const {
    RCLCPP_INFO(logger, "Printing events");
    for (auto &event : map_of_events) {
      RCLCPP_INFO(logger, "Event %d, start time %f, end time %f, power %f",
                  event.first, event.second.start_time, event.second.end_time,
                  event.second.power);
    }
  }
  /**
   * @brief Given an power draw modify the solution to allow for the acceptance
   * of the new event
   **/
  bool modify_solution_given_power(const float new_power, const int batt_id,
                                   const int start_idx, const int end_idx);
  inline double get_soc_for_slot(const int idx, const int batt_id) {
    return battery_id_to_energy_time_slots[batt_id][idx].energy;
  }
  /**
   * @brief This function returns the secondary fitness of the solution which is
   *the average soc
   **/
  inline double get_tertiary_fitness() const { return avg_soc; }
  inline double get_secondary_fitness() {
    return std::abs(total_max_power - total_min_power);
  }
  /**
   * @brief Return the size of the map of events
   **/
  inline int get_num_events() const { return map_of_events.size(); }
  /**
   * @brief Return the number of timesteps.
   **/
  inline int get_num_time_slots() const { return num_time_slots; }
  /**
   * @brief Return the number of failed events.
   **/
  inline unsigned long int get_num_failed_events() const {
    return failed_events;
  }
  /**
   * @brief This returns the power of the time slot with the given idx
   **/
  double get_power_for_time_slot(const int idx, const int battery_id) {
    return battery_id_to_energy_time_slots[battery_id][idx].power;
  }
  /**
   * @brief This prints the time slots for a specific battery.
   **/
  inline void print_time_slots(const int battery_id) {
    for (auto time_slot : battery_id_to_energy_time_slots[battery_id]) {
      std::cout << time_slot.to_string() << std::endl;
    }
  }
  /**
   * @brief This returns the idx of the time slot with the highest power.
   **/
  inline int get_max_time_slot_idx() {
    int max_idx = 0;
    double max_power = -1000000000;
    for (uint idx = 0; idx < battery_id_to_energy_time_slots[0].size(); ++idx) {
      double power_for_slot = external_source[idx].power;
      for (int i = 0; i < num_batteries; ++i) {
        power_for_slot += battery_id_to_energy_time_slots[i][idx].power;
      }
      if (power_for_slot > max_power) {
        max_power = power_for_slot;
        max_idx = idx;
      }
    }
    return max_idx;
  }
  /**
   * @brief This returns the idx of the time slot with the lowest power.
   **/
  inline int get_min_time_slot_idx() {
    int min_idx = 0;
    double min_power = 1000000000;
    for (uint idx = 0; idx < battery_id_to_energy_time_slots[0].size(); ++idx) {
      double power_for_slot = external_source[idx].power;
      for (int i = 0; i < num_batteries; ++i) {
        power_for_slot += battery_id_to_energy_time_slots[i][idx].power;
      }
      if (power_for_slot < min_power) {
        min_power = power_for_slot;
        min_idx = idx;
      }
    }
    return min_idx;
  }
  /**
   * @brief Prints all  time slots for all bateries
   **/
  inline void print_timeslot() {
    for (int i = 0; i < num_batteries; ++i) {
      std::cout << "Battery " << i << std::endl;
      for (auto timeslot : battery_id_to_energy_time_slots[i]) {
        std::cout << timeslot.to_string() << std::endl;
      }
    }
  }
  /**
   * @brief This function returns the total average power of the solution.
   **/
  inline double get_average_power() {
    double total_power = 0.0;
    for (int i = 0; i < num_time_slots; ++i) {
      total_power += external_source[i].power;
      for (int j = 0; j < num_batteries; ++j) {
        total_power += battery_id_to_energy_time_slots[j][i].power;
      }
    }
    return total_power / num_time_slots;
  }
  /**
   * @brief This function returns the primary objective and fills out the
   *seconday objective. The primary objective is the max power usage for the
   *system.
   **/
  inline double get_fitness() {
    // initialize the fit vale and soc
    double total_soc = 0;
    // initialize max power to the lowest possible value.
    double max_current_power = std::numeric_limits<double>::min();
    double min_current_power = std::numeric_limits<double>::max();
    // get the number of time slots
    const uint num_time_slots = battery_id_to_energy_time_slots[0].size();
    for (uint idx = 0; idx < num_time_slots; ++idx) {
      // initialize the power for the time slot with the external power
      double total_power_for_slot = external_source[idx].power;
      // loob through the batteries.
      for (int i = 0; i < this->num_batteries; ++i) {
        // get the power and soc of the battery for the time slot
        double power = battery_id_to_energy_time_slots[i][idx].power;
        double soc = battery_id_to_energy_time_slots[i][idx].energy;
        // add the soc to the total soc
        total_soc += soc;
        // add the power to the power of the time slot
        total_power_for_slot += power;
      }
      // check if the power for the slot is greater then the current max
      if (total_power_for_slot > max_current_power) {
        // reassign variables
        max_current_power = total_power_for_slot;
        this->max_idx = idx;
      }
      if (total_power_for_slot < min_current_power) {
        min_current_power = total_power_for_slot;
      }
    }
    this->total_min_power = min_current_power;
    this->total_max_power = max_current_power;
    this->avg_soc = total_soc / (num_time_slots * num_batteries);
    return max_current_power;
  }
  inline void print_max_idx_and_power() {
    RCLCPP_INFO(this->logger, "Max_idx: %d, Max_Power: %lf", this->max_idx,
                this->total_max_power);
  }
};
} // namespace ConnectedComm
#endif // BatteryAggregation_HPP
