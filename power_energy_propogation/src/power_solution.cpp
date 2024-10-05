#include "power_energy_propogation/power_solution.hpp"
#include "power_energy_propogation/data_struct.hpp"
#include <rclcpp/logging.hpp>

namespace ConnectedComm {
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
PowerSolution::PowerSolution(const int num_time_slots, const int num_batteries,
                             const float time_step,
                             const std::vector<double> load_profile,
                             const std::vector<double> batt_cap,
                             const std::vector<double> batt_rate,
                             const std::vector<double> batt_min,
                             const double soc_start)
    : time_step(time_step), failed_events(0), num_time_slots(num_time_slots),
      num_batteries(num_batteries), soc_start(soc_start) {
  RCLCPP_INFO(logger,
              "Creating Battery Aggregation with %d  batteries and "
              "%d number of time slots",
              num_batteries, num_time_slots);
  // loop through the number of batteries
  for (int i = 0; i < num_batteries; ++i) {
    // generate a battery object that holds the constraints for each battery
    const Battery battery(i, batt_cap[i], batt_rate[i], batt_min[i]);
    batteries.push_back(battery);
    RCLCPP_DEBUG(logger, "Creating Battery %d", i);
    // generate a vector of timeslots for each battery
    std::vector<EnergyTimeSlot> energy_time_slots;
    battery_id_to_energy_time_slots[i] = energy_time_slots;
  }
  // loop through the number of time slots
  for (int i = 0; i < num_time_slots; ++i) {
    // generate time slots for the external load
    EnergyTimeSlot energy_time_slot(i, batt_min[i]);
    energy_time_slot.power = load_profile[i];
    external_source.push_back(energy_time_slot);
    RCLCPP_DEBUG(logger, "Creating External Slot %d with power %lf ", i,
                 load_profile[i]);
  }
  // loop through the number of batteries and generate a time slot for each
  // battery
  for (int i = 0; i < num_batteries; ++i) {
    std::vector<EnergyTimeSlot> &energy_time_slots =
        battery_id_to_energy_time_slots[i];
    for (int j = 0; j < num_time_slots; ++j) {
      EnergyTimeSlot energy_time_slot(j, batt_min[j]);
      energy_time_slot.power = 0;
      energy_time_slot.energy = soc_start;
      energy_time_slots.push_back(energy_time_slot);
    }
  }
}
/**
 * @brief This function adds an event of interaction with a given battery.
 * An event is a time interval where the battery is being used as well as
 * a givens power demand of an event which is the product of the activity value
 * (which has a value between -1 and 1) and the battery rate.
 * @param start_time (double): Start time of the event.
 * @param end_time (double): End time of the event.
 * @param activity (double): Activity value of the event (-1 to 1 inclusive).
 * @param rate (double): Max Rate at which the the battery can charge and
 * discharge.
 **/
int PowerSolution::add_event(const double start_time, const double end_time,
                             const double activity, const double rate,
                             const int battery_id) {
  // Verify that the battery id exists
  if (battery_id_to_energy_time_slots.find(battery_id) ==
      battery_id_to_energy_time_slots.end()) {
    std::cout << "Battery id " << battery_id << " does not exist." << std::endl;
    throw std::logic_error("Battery id does not exist.");
  } else {
    // Get the time slot that the event starts in
    const int start_floor_time = std::floor(start_time);
    RCLCPP_DEBUG(logger, "Start Time: %lf, Start Floor Time: %d", start_time,
                 start_floor_time);
    // Get the percentage of the time slot that the event occurs in

    // Get the time slot that the event ends in
    const int end_floor_time = std::floor(end_time);
    // Get the percentage of the time slot that the event occurs in
    // Get the power of the event
    const double new_power = activity * rate;
    RCLCPP_DEBUG(logger, "Activity: %f, Rate: %f, New Power: %f", activity,
                 rate, new_power);

    // Create a new event
    const int event_id = this->map_of_events.size();
    RCLCPP_DEBUG(logger, "Creating Event: %d", event_id);
    Event new_event(event_id, battery_id, activity, rate, start_time, end_time);
    this->map_of_events[event_id] = new_event;
    // Loop through the time slots that the event occurs in
    bool soc_violated = false;
    std::vector<EnergyTimeSlot> energy_time_slot_copy =
        this->battery_id_to_energy_time_slots[battery_id];
    bool passed_event_times = false;
    RCLCPP_DEBUG(logger, "Looping from %d to %ld with New Power: %f",
                 start_floor_time, energy_time_slot_copy.size(), new_power);

    const int size_of_time_slots =
        static_cast<int>(energy_time_slot_copy.size()) - 1;
    for (int time = start_floor_time; time <= size_of_time_slots; ++time) {
      // Energy Propogation -----------
      // if the time slot is not the first slot
      if (time != 0) {
        // get the previous and current energy time slot
        EnergyTimeSlot &prev_energy_time = energy_time_slot_copy[time - 1];
        RCLCPP_DEBUG(logger, "Previous energy: %lf", prev_energy_time.energy);
        EnergyTimeSlot &energy_time = energy_time_slot_copy[time];
        // add the energy from the previous time slot pluss the
        // accumlated charge between the two times
        const double prev_energy = prev_energy_time.energy;
        energy_time.energy = prev_energy_time.energy +
                             (prev_energy_time.power * time_step / 60.0);
        if (energy_time.energy > prev_energy) {
        } else {
        }
        if ((energy_time.energy > batteries[battery_id].capacity) or
            (energy_time.energy < batteries[battery_id].minimum)) {
          soc_violated = true;
          RCLCPP_ERROR(logger, "SOC violated: %lf at time %d",
                       energy_time.energy, time);
          break;
        } // add the power of the event

        if (!passed_event_times) {
          energy_time.power += new_power;
          RCLCPP_DEBUG(logger, "new power: %lf, event: %d, time: %d",
                       energy_time.power, event_id, time);
          if ((energy_time.power > batteries[battery_id].max_rate) or
              (energy_time.power < -batteries[battery_id].max_rate)) {
            soc_violated = true;
            RCLCPP_ERROR(logger, "Power violated: %lf at time %d",
                         energy_time.power, time);
            break;
          }
        }
        if (time >= end_floor_time) {
          passed_event_times = true;
        }
        // if the time slot is the first slot only the power needs
        // to be incremented
      } else {
        RCLCPP_DEBUG(logger, "At start time");
        EnergyTimeSlot &energy_time = energy_time_slot_copy[time];
        energy_time.power += new_power;
      }
    }
    if (!soc_violated) {
      RCLCPP_DEBUG(logger, "Solution Accepted");
      battery_id_to_energy_time_slots[battery_id].clear();
      std::copy(
          energy_time_slot_copy.begin(), energy_time_slot_copy.end(),
          std::back_inserter(battery_id_to_energy_time_slots[battery_id]));
      return event_id;
    } else {
      failed_events += 1;
      this->map_of_events.erase(event_id);
      return -1;
    }
  }
}
std::tuple<double, double>
PowerSolution::get_bounds_of_problem(const int id, const int start_idx,
                                     const int end_idx) {
  // int variables that will be used to construct bounds on activity to be added
  double max_soc = 0.0;
  double min_soc = 100000;
  double max_power = 0.0;
  double min_power = 100000;
  RCLCPP_DEBUG(logger, "Size of battery %d: %ld", id,
               battery_id_to_energy_time_slots[id].size());
  // loop through all time slots from the start idx to the end of the time
  // horizon
  for (int i = start_idx; i < num_time_slots; ++i) {
    // get the energy and power of the slot
    const double power = battery_id_to_energy_time_slots[id][i].power;
    const double energy = battery_id_to_energy_time_slots[id][i].energy;
    // update the energy variables
    if (energy > max_soc) {
      max_soc = energy;
    } else if (energy < min_soc) {
      min_soc = energy;
    }
    // if the time is before the end_idx update the power variables
    if (power > max_power and i < end_idx) {
      max_power = power;
    } else if (power < min_power and i < end_idx) {
      min_power = power;
    }
  }
  // get the bound of the of the activity scalar for each variable
  const double diff_max_power =
      (batteries[id].max_rate - max_power) / batteries[id].max_rate;
  const double diff_min_power =
      (-batteries[id].max_rate - min_power) / batteries[id].max_rate;
  const double diff_max_soc =
      (batteries[id].capacity - max_soc) /
      (batteries[id].max_rate * batteries[id].capacity * (time_step / 3600));
  const double diff_min_soc =
      (batteries[id].minimum - min_soc) /
      (batteries[id].max_rate * batteries[id].capacity * (time_step / 3600));
  RCLCPP_DEBUG(logger,
               "Max power: %lf, Min power %lf, Max SOC %lf, Min SOC %lf",
               max_power, min_power, max_soc, min_soc);
  RCLCPP_DEBUG(logger, "Max rate: %lf, Capacity %lf, Min SOC %lf",
               batteries[id].max_rate, batteries[id].capacity,
               batteries[id].minimum);
  RCLCPP_DEBUG(logger, "Power max %lf, Power min %lf, SOC max %lf, SOC min %lf",
               diff_max_power, diff_min_power, diff_max_soc, diff_min_soc);
  // get the max and min of the activity scalar
  double max_act = std::min(diff_max_power, diff_max_soc);
  double min_act = std::max(diff_min_power, diff_min_soc);
  max_act = std::min(max_act, 1.0);
  min_act = std::max(min_act, -1.0);
  const auto return_tuple = std::make_tuple(min_act, max_act);
  return return_tuple;
}
/**
 * @brief Given an power draw modify the solution to allow for the acceptance
 * of the new event
 **/
bool PowerSolution::modify_solution_given_power(const float new_power,
                                                const int batt_id,
                                                const int start_idx,
                                                const int end_idx) {
  // cast the end idx to a uint to allow comparison
  const uint end_idx_uint = static_cast<uint>(end_idx);
  // initialilze previous power and idx to allow for propogation
  // initalize a reference to the battery utilized in the current event
  const Battery &battery = batteries[batt_id];
  std::vector<EnergyTimeSlot> time_slot_copy(
      this->battery_id_to_energy_time_slots[batt_id]);
  float prev_power;
  float prev_soc;
  if (start_idx == 0) {
    prev_power = 0.0;
    prev_soc = 0.0;
  } else {
    prev_power = time_slot_copy[start_idx - 1].power;
    prev_soc = time_slot_copy[start_idx - 1].energy;
  }
  const uint num_time_slots = time_slot_copy.size();
  // loop for the start of the event to the end of the time slots
  for (uint i = start_idx; i < num_time_slots; ++i) {
    // Grab the time slot for the battery
    EnergyTimeSlot &time_slot = time_slot_copy[i];
    // calculate the amout of power and energy if the event is added
    float mod_power = time_slot.power + new_power;
    float new_energy = prev_power * time_step / 60.0 + prev_soc;
    // if before the end of the event the power is modified
    if (i < end_idx_uint) {
      // if the power exceeds the allowed rate
      if (mod_power > battery.max_rate) {
        time_slot.power = battery.max_rate;
        mod_power = battery.max_rate;
        // if the power is below the discharge rate
      } else if (mod_power < -battery.max_rate) {
        time_slot.power = -battery.max_rate;
        mod_power = -battery.max_rate;
      } else {
        time_slot.power = mod_power;
      }
    }
    // if the new energy violates max soc
    if (new_energy > battery.capacity or new_energy < battery.minimum) {
      // calculate the needed power to allow for the event
      if (new_energy > battery.capacity) {
        new_energy = battery.capacity;
      } else {
        new_energy = battery.minimum;
      }
      time_slot.energy = new_energy;
      double power_to_validate =
          (60 * (battery.capacity - prev_soc)) / time_step;
      // remove the power from the previous time slot
      prev_power = power_to_validate;
      if (prev_power > -battery.max_rate and prev_power < battery.max_rate) {
        battery_id_to_energy_time_slots[batt_id][i - 1].power =
            power_to_validate;
        // if the new power will invalidate the power rate
      } else {
        RCLCPP_ERROR(this->logger,
                     "Solution Modification Failed power to validate is: %lf, "
                     "prev_soc: %lf, new_power: %lf",
                     power_to_validate, prev_soc, new_power);
        return false;
      }
    }
    // assign previous values
    prev_power = mod_power;
    prev_soc = new_energy;
  }
  battery_id_to_energy_time_slots[batt_id] = std::move(time_slot_copy);
  return true;
}
} // end namespace ConnectedComm
