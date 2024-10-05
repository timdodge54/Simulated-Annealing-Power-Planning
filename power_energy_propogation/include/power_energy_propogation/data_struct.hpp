#ifndef DATA_STRUCT_HPP
#define DATA_STRUCT_HPP
#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ConnectedComm {
/** @struct Event
 *  @brief Class to hold event information.
 *  @param id (int): Event id.
 *  @param power (float): Charge of the event.
 *  @param start_time(int): Start time of the event.
 *  @param end_time(int): End time of the event.
 */
struct Event {
  int id; // Event id
  int battery_id;
  double power;      // Energy consumed by the event at a full time slot
  double start_time; // start time of the event
  double activity;   //
  double rate;       //
  double end_time;   // end time of the event

  /** @brief Constructor for the Event class.
   *
   * @param id (int): Event id.
   * @param power (float): Charge of the event.
   * @param start_time(int): Start time of the event.
   * @param end_time(int): End time of the event.
   */
  Event() = default;
  Event(int id, int battery_id, double activity, double rate, double start_time,
        double end_time) {
    this->id = id;
    this->activity = activity;
    this->rate = rate;
    this->power = activity * rate;
    this->start_time = start_time;
    this->end_time = end_time;
    this->battery_id = battery_id;
  }
  /** @brief Function to print the event information.
   * @return (std::string): String containing the event information.
   */
  inline std::string to_string() const {
    return "Id " + std::to_string(id) + ", Charge: " + std::to_string(power) +
           ", Start: " + std::to_string(start_time) +
           ", End: " + std::to_string(end_time);
  }
};

/** @struct EnergyTimeSlot
 *  @brief Class to hold event information.
 *  @param time_idx (int):
 *  @param power (float): Charge of the event.
 *  @param start_time(int): Start time of the event.
 *  @param end_time(int): End time of the event.
 */
struct EnergyTimeSlot {
  int time_idx;
  double energy;
  double power;
  double power_rate;
  std::vector<int> event_ids;
  EnergyTimeSlot(int time_idx, double power_rate)
      : time_idx(time_idx), energy(0.0), power(0.0), power_rate(power_rate) {}
  inline std::string to_string() const {

    std::string str = "Time: " + std::to_string(this->time_idx) +
                      ", Energy: " + std::to_string(this->energy) +
                      ", Power: " + std::to_string(this->power);
    for (auto event_id : this->event_ids) {
      str += ", Event: " + std::to_string(event_id);
    }
    return str;
  }
};

struct Battery {
  int id;          // Id of battery
  double capacity; // Soc max
  double max_rate; // max power allowable
  double minimum;  // Soc min
  Battery(int id, double capacity, double max_rate, double minimum)
      : id(id), capacity(capacity), max_rate(max_rate), minimum(minimum) {}
  inline std::string to_string() const {
    std::string str = "Battery: " + std::to_string(this->id) +
                      ", Capacity: " + std::to_string(this->capacity) +
                      ", Max Rate: " + std::to_string(this->max_rate) +
                      ", Minimum: " + std::to_string(this->minimum);
    return str;
  }
};

enum planner_type {
  GREEDY,
  SA,
};

} // namespace ConnectedComm
#endif // DATA_STRUCT_HPP
