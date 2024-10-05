#include "power_energy_propogation/planner.hpp"
#include <random>
#include <rclcpp/logging.hpp>

namespace ConnectedComm {
/*
 * @brief: This is the constructor for the simulated anneahling object that
 * houses both a greedy algorithm and a simulated anneahling algorithm.
 * @param (int) num_time_slots: The number of time slots for the simulation
 * @param (int) num_batteries: The number of batteries in the simulation
 * @param (float) time_step: The time step for the simulation
 * @param (std::vector<double>) load_profile: The load profile for the
 * @param (double) soc_min: The minimum state of charge for the batteries
 * @param (double) soc_max: The maximum state of charge for the batteries
 * @param (double) temperture: The initial temperature for the simulated
 * @param (int) max_iterations: The maximum number of iterations for the
 */
Planner::Planner(const int num_time_slots, const int num_batteries,
                 const float time_step, const std::vector<double> load_profile,
                 const std::vector<double> batt_cap,
                 const std::vector<double> batt_rate,
                 const std::vector<double> batt_min, const double temperture,
                 const int max_iterations, const planner_type plan,
                 const double temp_cooling_rate, const double soc_start,
                 const double temp_min, const double fitness_scalar)
    : previous_max(-111111), logger(rclcpp::get_logger("Planner")),
      best_cost(10000000), battery_rates(batt_rate), temperature(temperture),
      max_iterations(max_iterations), num_failed_events(0), num_violations(0),
      num_batteries(num_batteries),
      seed(std::chrono::system_clock::now().time_since_epoch().count()),
      temp_cooling_rate(temp_cooling_rate), soc_start(soc_start),
      temp_min(temp_min), fitness_scalar(fitness_scalar) {
  // genrate the initial solution
  curr_sol =
      PowerSolution(num_time_slots, num_batteries, time_step, load_profile,
                    batt_cap, batt_rate, batt_min, soc_start);
  RCLCPP_INFO(logger, "Temp Cooling Rate: %f", temp_cooling_rate);
  // generate the random engine and seed with a random seed
  this->re = std::default_random_engine(seed);
  this->gen = std::mt19937(seed);
  // get the current solutions major fitness
  this->best_cost = curr_sol.get_fitness();
  this->curr_sol_cost = best_cost;
  this->best_sol = curr_sol;
  // check the planner type and execute the appropriate algorithm
  if (plan == planner_type::GREEDY) {
    // set the logger
    RCLCPP_INFO(logger, "Using Greedy Planner");
    this->greedy_loop();
    this->best_sol = this->curr_sol;
  } else if (plan == planner_type::SA) {
    // set the logger
    this->SA_loop();
  }
  const double avg = curr_sol.get_average_power();
  RCLCPP_INFO(logger, "Average Power: %lf", avg);
}
/*
 * @brief: This is the main function for the simulated anneahling algorithm.
 */
void Planner::greedy_loop() {
  for (int iter = 0; iter < this->max_iterations; ++iter) {
    this->perturbate_solution_greedy(iter);
  }
  RCLCPP_INFO(logger, "The number of failed event is: %d", num_failed_events);
  RCLCPP_INFO(logger, "The number of violations is: %d", num_violations);
}
/**
 * @brief: The loop that executes a random perturbation performs the sa
 *acceptance and decrements the temperature.
 **/
void Planner::SA_loop() {
  RCLCPP_INFO(logger,
              "The cooling rate is: %lf, temp_max is: %lf, temp_min is: %lf",
              temp_cooling_rate, temperature, temp_min);
  // set the initial temperature for the Tfactor
  const double temp_0 = temperature;
  for (int i = 0; i < max_iterations; ++i) {
    // perturbate the solution and check acceptance
    this->perturbate_solution_SA(i);
    // decrement the temperature
    const double Tfactor = -std::log(temp_0 / temp_min);
    this->temperature = temp_0 * std::exp((Tfactor * i) / max_iterations);
    if (i % 10 == 0) {
      RCLCPP_WARN(logger, "Iteration: %d", i);
      RCLCPP_WARN(logger, "The temperature is: %lf", temperature);
    }
  }
  RCLCPP_INFO(logger, "The number of failed event is: %d", num_failed_events);
  RCLCPP_INFO(logger, "The number of violations is: %d", num_violations);
  // set the current solution to the best solution
  this->curr_sol = this->best_sol;
}

bool Planner::generate_random_solution() {
  PowerSolution prev_sol = this->curr_sol;
  RCLCPP_INFO(logger, "Generating Random Solution");
  std::uniform_int_distribution<int> unifid(0, num_batteries - 1);
  const int id = unifid(re);
  // get the number of time slots and cast to a double
  const double total_slots = static_cast<double>(curr_sol.get_num_time_slots());
  // get the power rating of the battery
  const double power = battery_rates[id];
  // Initialize event parameters
  double start_time, end_time, lower, upper;
  // loop until valid activity constraints are found
  int counter_falied = 0;
  while (true) {
    // generate a random start time
    std::uniform_real_distribution<double> unif3(0, total_slots);
    start_time = unif3(re);
    // generate a random end time
    std::uniform_real_distribution<double> unif4(start_time + 1.0, total_slots);
    end_time = unif4(re);
    // get the valid upper and lower bounds for the power to be added to the
    // solution
    const int start_time_idx = std::floor(start_time);
    const int end_time_idx = std::floor(end_time);
    std::tuple<double, double> soc_violated =
        curr_sol.get_bounds_of_problem(id, start_time_idx, end_time_idx);
    // Get the upper and lower bounds
    lower = std::get<0>(soc_violated);
    upper = std::get<1>(soc_violated);
    // if the lower bound is less than the upper bound then break
    if (lower < upper or lower == upper) {
      break;
    }
    if (counter_falied % 100 == 0 and std::abs(upper - lower) > 0.0001) {
      RCLCPP_INFO(logger, "The lower is %lf and the upper is %lf", lower,
                  upper);
    }
    curr_sol = prev_sol;
    counter_falied++;
  }
  // divde the bounds by 3 to make the boundries of the problem 3 sigma distance
  const double lower_cut = lower / 3;
  const double upper_cut = upper / 3;
  // get the mean of the bounds
  const double inbetween = (upper_cut + lower_cut) / 2;
  std::normal_distribution<double> norm;
  RCLCPP_INFO(logger, "Lower cut %lf, Upper cut %lf, Inbetween %lf", lower_cut,
              upper_cut, inbetween);
  double act;
  // loop until activity scalar is within bounds
  counter_falied = 0;
  while (true) {
    // get a random activity scalar with center at average and sigma of 3rd of
    // upper bound
    norm = std::normal_distribution<double>(inbetween, upper_cut);
    norm(gen);
    act = norm(re);
    // verify that the activity scalar is within bounds
    if (act < upper && act > lower) {
      break;
    } else if (counter_falied % 100 == 0 && std::abs(upper - lower) < 0.0001) {
      RCLCPP_WARN(logger, "The event failed with lower %lf and upper %lf",
                  lower, upper);
      num_failed_events++;
      num_violations++;
      return false;
    }
  }
  const int event_id =
      this->curr_sol.add_event(start_time, end_time, act, power, id);
  if (event_id == -1) {
    RCLCPP_INFO(logger, "Event Failed: from %lf to %lf, with power of %lf",
                start_time, end_time, act * power);
    num_failed_events++;
    num_violations++;
    return false;
  }
  RCLCPP_INFO(logger, "Event from %lf to %lf, with power of %lf", start_time,
              end_time, act * power);
  return true;
}
bool Planner::force_solution() {
  RCLCPP_INFO(logger, "Forcing Solution");
  while (true) {
    std::uniform_int_distribution<int> unifid(0, num_batteries - 1);
    const int id = unifid(re);
    // get the number of time slots and cast to a double
    const double total_slots =
        static_cast<double>(curr_sol.get_num_time_slots());
    // get the power rating of the battery
    const double power = battery_rates[id];
    // Initialize event parameters
    std::uniform_real_distribution<double> unif(0, total_slots);
    double start_time = unif(re);
    // loop until valid activity constraints are found
    std::uniform_real_distribution<double> unif1(start_time, total_slots);
    double end_time = unif1(re);
    std::normal_distribution<double> norm(0, .3);
    double act;
    while (true) {
      act = norm(re);
      if (act > -1 && act < 1) {
        break;
      }
    }
    const int start_idx = std::floor(start_time);
    const int end_idx = std::floor(end_time);
    const double used_power = power * act;
    bool valid = this->curr_sol.modify_solution_given_power(used_power, id,
                                                            start_idx, end_idx);
    if (valid) {
      RCLCPP_DEBUG(logger, "Event from %lf to %lf, with power of %lf",
                   start_time, end_time, used_power);
      return true;
    }
  }
}
/*
 * @brief: This generates a random solution checks if it fails if it does not
 * if it is better than the previous solution it is accepted.
 */
bool Planner::geneate_random_solution_early_peak_hit() {
  RCLCPP_INFO(logger, "Generating Random Solution");
  std::uniform_int_distribution<int> unifid(0, num_batteries - 1);
  PowerSolution prev_sol = curr_sol;
  const int id = unifid(re);
  // get the number of time slots and cast to a double
  const double total_slots = static_cast<double>(curr_sol.get_num_time_slots());
  // get the power rating of the battery
  const double power = battery_rates[id];
  // Initialize event parameters
  double start_time, end_time, lower, upper;
  // loop until valid activity constraints are found
  int max_idx = curr_sol.get_max_time_slot_idx();
  start_time = max_idx - 100;
  if (start_time < 0) {
    start_time = 0;
  }
  double end_slot = start_time + 101;
  if (end_slot > total_slots) {
    end_slot = static_cast<double>(total_slots);
  }
  int counter_failed = 0;
  while (true) {
    // generate a random end time
    std::uniform_real_distribution<double> unif4(end_slot, total_slots);
    end_time = unif4(re);
    // get the valid upper and lower bounds for the power to be added to the
    // solution
    const int start_time_idx = std::floor(start_time);
    const int end_time_idx = std::floor(end_time);
    std::tuple<double, double> soc_violated =
        curr_sol.get_bounds_of_problem(id, start_time_idx, end_time_idx);
    // Get the upper and lower bounds
    lower = std::get<0>(soc_violated);
    upper = 0;
    // if the lower bound is less than the upper bound then break
    if (lower < upper) {
      break;
    }
    curr_sol = prev_sol;
    counter_failed++;
    if (counter_failed % 100 == 0 && std::abs(lower - upper) < .00001) {
      RCLCPP_WARN(logger, "Failed to find a valid event with %lf to %lf", lower,
                  upper);
      num_failed_events++;
      num_violations++;
      return false;
    }
  }
  // get the mean of the bounds
  double act;
  // loop until activity scalar is within bounds
  // get a random activity scalar with center at average and sigma of 3rd of
  // upper bound
  std::uniform_real_distribution<double> random_draw(lower, upper);
  act = random_draw(re);
  // verify that the activity scalar is within bounds
  const int event_id =
      this->curr_sol.add_event(start_time, end_time, act, power, id);
  if (event_id == -1) {
    num_failed_events++;
    num_violations++;
    return false;
  }
  RCLCPP_INFO(logger, "Event from %lf to %lf, with power of %lf", start_time,
              end_time, act * power);
  return true;
}
bool Planner::force_solution_hit_peak() {
  int start_idx = this->curr_sol.get_max_time_slot_idx();
  this->curr_sol.print_max_idx_and_power();
  RCLCPP_INFO(logger, "Forcing Solution Hit Peak");
  int count = 0;
  while (true) {
    if (count > 100) {
      RCLCPP_WARN(logger, "Cant hit peak.");
      return this->force_solution();
    }
    std::uniform_int_distribution<int> unifid(0, num_batteries - 1);
    const int id = unifid(re);
    // get the number of time slots and cast to a double
    const double total_slots =
        static_cast<double>(curr_sol.get_num_time_slots());
    // get the power rating of the battery
    const double power = battery_rates[id];
    // Initialize event parameters
    // loop until valid activity constraints are found
    std::uniform_real_distribution<double> unif1(start_idx, total_slots);
    double end_time = unif1(re);
    std::uniform_real_distribution<double> unif2(-.1, 0);
    double act = unif2(re);
    const int end_idx = std::floor(end_time);
    const double used_power = power * act;
    bool valid = this->curr_sol.modify_solution_given_power(used_power, id,
                                                            start_idx, end_idx);
    if (valid) {
      RCLCPP_INFO(logger, "Event from %d to %lf, with power of %lf", start_idx,
                  end_time, used_power);
      return true;
    }
    count++;
  }
}
/*
 * @brief: This generates a random solution checks if it fails if it does not
 * if it is better than the previous solution it is accepted.
 */
void Planner::perturbate_solution_greedy(int iter) {
  // get the current solution to compare to
  PowerSolution prev_sol = curr_sol;
  std::uniform_real_distribution<double> rand(0, 1);
  double rand_num = rand(re);
  std::unique_ptr<std::tuple<double, double, double, double>> event_params;
  bool valid = false;
  double percent_iter = static_cast<double>(iter) / max_iterations;
  if (percent_iter < .1) {
    valid = this->geneate_random_solution_early_peak_hit();
    // add the event to the solution
  } else if (rand_num > .3) {
    valid = this->generate_random_solution();
    // add the event to the solution
  } else if (rand_num > .15) {
    valid = this->force_solution();
  } else {
    valid = this->force_solution_hit_peak();
  }
  // check if the event failed
  if (!valid) {
    RCLCPP_DEBUG(logger, "EVENT FAILED:");
    // iterate the number of failed events and violations
    curr_sol = prev_sol;
    return;
  }
  // check if the solution is better than the previous solution
  if (greedy_accept_new_sol(prev_sol, curr_sol)) {
    RCLCPP_DEBUG(logger, "Greedy Accept New Solution");
  } else {
    RCLCPP_DEBUG(logger, "Solution Not Accepted");
    // iterate the number of failed events
    num_failed_events++;
    // revert the solution
    curr_sol = prev_sol;
  }
  return;
}
/*
 * @brief: This generates a random solution checks if it fails if it does not
 * if it is better than the previous solution it is accepted.
 */
void Planner::perturbate_solution_SA(int iter) {
  // get the current solution to compare to
  auto prev_sol = curr_sol;
  // generate random event parameters
  std::uniform_real_distribution<double> rand(0, 1);
  double rand_num = rand(re);
  std::unique_ptr<std::tuple<double, double, double, double>> event_params;
  double percent_iter = static_cast<double>(iter) / max_iterations;
  bool valid = false;
  if (percent_iter <= .2) {
    valid = this->geneate_random_solution_early_peak_hit();
  }
  if (rand_num > .3) {
    valid = this->generate_random_solution();
  } else if (rand_num > .15) {
    valid = this->force_solution();
  } else {
    valid = this->force_solution_hit_peak();
  }
  // check if the event failed
  if (!valid) {
    RCLCPP_DEBUG(logger, "EVENT FAILED:");
    num_violations++;
    num_failed_events++;
    return;
  }
  // check acceptance of the new solution with simulated annealing
  if (accept_solution_SA(prev_sol, curr_sol, temperature)) {
    RCLCPP_DEBUG(logger, "Simulated Accept New Solution");
    compare_best();
  } else {
    RCLCPP_DEBUG(logger, "Simulated Solution Not Accepted.");
    num_failed_events++;
    curr_sol = prev_sol;
  }
  return;
}
/*
 * @brief: Greedy acceptance
 * @param: new_power_max: The new max power
 * @param: prev_power_max: The previous max power
 * @return: True if the new power is less than the previous power
 */

bool Planner::accept_solution_SA(PowerSolution &prev_sol,
                                 PowerSolution &new_sol, double temp) {
  // get primary fitness of both solutions
  const double prev_fit = prev_sol.get_fitness();
  const double curr_fit = new_sol.get_fitness();
  const double prev_secondary_cost = prev_sol.get_secondary_fitness();
  const double curr_secondary_cost = new_sol.get_secondary_fitness();
  const double prev_tertiary_cost = prev_sol.get_tertiary_fitness();
  const double curr_tertiary_cost = new_sol.get_tertiary_fitness();
  RCLCPP_DEBUG(logger, "Previous Fitness: %lf, Current Fitness: %lf", prev_fit,
               curr_fit);
  // if the primary fitness is lower the the old accept the solution
  bool is_tertiary;
  if (prev_fit > curr_fit) {
    curr_sol_cost = curr_fit;
    RCLCPP_DEBUG(logger, "Solution Accepted");
    RCLCPP_DEBUG(logger, "Best Cost: %lf", best_cost);
    return true;
    // if the primary fitness is the same check the secondary fitness
  } else if (prev_fit == curr_fit) {
    // if the secondary fitness is lower accept the solution
    if (prev_secondary_cost > curr_secondary_cost) {
      curr_sol_cost = curr_fit;
      RCLCPP_INFO(logger, "Solution Accepted Strict.");
      return true;
    } else if (prev_secondary_cost == curr_secondary_cost) {
      if (prev_tertiary_cost < curr_tertiary_cost) {
        curr_sol_cost = curr_fit;
        return true;
      } else {
        RCLCPP_INFO(logger, "Secondary fitnees check..");
        is_tertiary = true;
        const bool accept = this->check_solutions_SA(
            prev_tertiary_cost, curr_tertiary_cost, temp, is_tertiary);
        return accept;
      }

    } else {
      RCLCPP_INFO(logger, "Secondary fitnees check..");
      is_tertiary = false;
      const bool accept = this->check_solutions_SA(
          prev_secondary_cost, curr_secondary_cost, temp, is_tertiary);
      return accept;
    }
  } else {
    RCLCPP_INFO(logger, "Primary fitnees check..");
    is_tertiary = false;
    const bool accept =
        this->check_solutions_SA(prev_fit, curr_fit, temp, is_tertiary);
    return accept;
  }
}

} // namespace ConnectedComm
