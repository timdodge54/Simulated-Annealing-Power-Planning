#ifndef Planner_HPP
#define Planner_HPP
#include "power_energy_propogation/data_struct.hpp"
#include "power_energy_propogation/power_solution.hpp"
#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <random>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace ConnectedComm {
/*
 * @brief: This is the class Planning Object Class. It holds both the greedy
 * planner and the simulated anneahling planner.
 * @attr (PowerSolution) curr_sol: The battery aggregate that propogates the
 * power and energy, holds the events, and tracks the maximum power time index.
 * @attr (double) previous_max: The previous maximum power
 * @attr (double) best_cost: The best cost of the solution
 * @attr (std::vector<EnergyTimeSlot>) best_solution: The best solution
 * @attr (std::map<int, Event>) best_event_map: The best event map
 * @attr (std::vector<double>) solution_fitness_history: The history of the
 * @attr (double) temperature: The temperature for the simulated anneahling
 * @attr (int) max_iterations: The maximum number of iterations for the
 * @attr (int) num_failed_events: The number of failed events
 * @attr (std::default_random_engine) re: The random engine for the simulated
 */
class Planner {
private:
  double previous_max;                       // the previous maximum power
  rclcpp::Logger logger;                     // the ros logger
  double best_cost;                          // the overall best cost
  std::vector<EnergyTimeSlot> best_solution; // the vector of the best solution
  std::map<int, Event> best_event_map; // map of events for the best solution
  std::vector<double> solution_fitness_history; // the history of best solutions
  std::vector<double> battery_rates;            // the battery rates
  double temperature;    // the current temperature of the simulated annealing
                         // algorithm
  int max_iterations;    // the max number of iterations regardless of the
                         // algorithm
  int num_failed_events; // the number of failed events
  int num_violations;    // the number of events that violate the constraints
  int num_batteries;     // the number of batteries
  unsigned int seed;     // the seed for the random number generator

  std::default_random_engine
      re;                   // the random engine for the simulated annealing
  std::mt19937 gen;         // the random engine for the simulated annealing
  double temp_cooling_rate; // the cooling rate for the linear cooling schedule
  double curr_sol_cost;     // the current solution cost
  double soc_start;         // the starting state of charge for the batteries
  double temp_min;       // the minimum temperature for the simulated annealing
  double fitness_scalar; // the scalar for the fitness function
  std::vector<double>
      primary_fitness_history; // the history of best primary fitness history
  std::vector<double>
      secondary_fitness_history; // the history of best secondary fitness
  std::vector<double>
      tertiary_fitness_history; // the history of best tertiary fitness

public:
  PowerSolution curr_sol; // the current solution
  PowerSolution best_sol; // the best solution
  Planner() : logger(rclcpp::get_logger("planner")){};
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
  Planner(const int num_time_slots, const int num_batteries,
          const float time_step, const std::vector<double> load_profile,
          const std::vector<double> batt_cap,
          const std::vector<double> batt_rate,
          const std::vector<double> batt_min, const double temperture = 200.0,
          const int max_iterations = 1000,
          const planner_type plan = planner_type::SA,
          const double temp_cooling_rate = .99, const double soc_start = 150.0,
          const double temp_min = 2.0, const double fitness_scalar = 250.0);
  std::vector<double> get_primary_history() {
    return this->primary_fitness_history;
  }
  std::vector<double> get_secondary_history() {
    return this->secondary_fitness_history;
  }
  std::vector<double> get_tertiary_history() {
    return this->tertiary_fitness_history;
  }
  /*
   * @brief: This is the main function for the simulated anneahling algorithm.
   */
  void greedy_loop();
  /*
   * @brief: This generates a random solution checks if it fails if it does not
   * if it is better than the previous solution it is accepted.
   */
  bool generate_random_solution();
  /*
   * @brief: This generates a random solution checks if it fails if it does not
   * if it is better than the previous solution it is accepted.
   */
  bool geneate_random_solution_early_peak_hit();
  /*
   * @brief: This generates modifies the current solution to accept an event.
   */
  bool force_solution();
  /*
   * @brief: This generates modifies the current solution to accept an event
   * that starts at the peak.
   */
  bool force_solution_hit_peak();

  /*
   * @brief Perturbate the solution then calls a greedy acceptance function
   **/
  void perturbate_solution_greedy(int iter);
  /*
   * @brief Perturbates the solution then calls a simulated anneahling
   **/
  void perturbate_solution_SA(int iter);
  bool accept_solution_SA(PowerSolution &prev_sol, PowerSolution &new_sol,
                          double temp);
  /*
   * @brief: Greedy acceptance
   */
  inline bool greedy_accept_new_sol(PowerSolution &prev_sol,
                                    PowerSolution &new_sol) {
    const double prev_fitness = prev_sol.get_fitness();
    const double curr_fitness = new_sol.get_fitness();
    const double prev_secondary_fitness = prev_sol.get_secondary_fitness();
    const double curr_secondary_fitness = new_sol.get_secondary_fitness();
    const double prev_tertiary_fitness = prev_sol.get_tertiary_fitness();
    const double curr_tertiary_fitness = new_sol.get_tertiary_fitness();
    if (prev_fitness > curr_fitness) {
      RCLCPP_INFO(this->logger,
                  "Accepting primary fitness with new sol %lf compared to %lf",
                  curr_fitness, prev_fitness);
      this->primary_fitness_history.push_back(curr_fitness);
      this->secondary_fitness_history.push_back(curr_secondary_fitness);
      this->tertiary_fitness_history.push_back(curr_tertiary_fitness);
      return true;
    } else if (prev_fitness == curr_fitness) {
      if (prev_secondary_fitness > curr_secondary_fitness) {
        RCLCPP_INFO(this->logger,
                    "Accepting secondary fitness with primary fitness of %lf "
                    "compared to "
                    "%lf, and secondary fitness of %lf compared to %lf",
                    curr_fitness, prev_fitness, curr_secondary_fitness,
                    prev_secondary_fitness);
        this->primary_fitness_history.push_back(curr_fitness);
        this->secondary_fitness_history.push_back(curr_secondary_fitness);
        this->tertiary_fitness_history.push_back(curr_tertiary_fitness);
        return true;
      } else if (prev_secondary_fitness == curr_secondary_fitness) {
        if (prev_tertiary_fitness < curr_tertiary_fitness) {
          this->primary_fitness_history.push_back(curr_fitness);
          this->secondary_fitness_history.push_back(curr_secondary_fitness);
          this->tertiary_fitness_history.push_back(curr_tertiary_fitness);
        }
      }
    }

    return false;
  }
  /**
   * @brief: The loop that executes a random perturbation performs the sa
   *acceptance and decrements the temperature.
   **/
  void SA_loop();
  /**
   * @brief: Checks if the current solution is better than then
   *the best solution.
   **/
  inline void compare_best() {
    double curr_sol_cost = curr_sol.get_fitness();
    double best_sol_cost = best_sol.get_fitness();
    double curr_sol_secondary_fitness = curr_sol.get_secondary_fitness();
    double best_sol_secondary_fitness = best_sol.get_secondary_fitness();
    double curr_sol_tertiary_fitness = curr_sol.get_tertiary_fitness();
    double best_sol_tertiary_fitness = best_sol.get_tertiary_fitness();
    if (best_sol_cost > curr_sol_cost) {
      RCLCPP_INFO(logger, "New Best Solution Found, Prev Cost: %lf, Cost: %lf",
                  best_cost, curr_sol_cost);
      best_sol = curr_sol;
      best_cost = curr_sol_cost;
      primary_fitness_history.push_back(curr_sol_cost);
      secondary_fitness_history.push_back(curr_sol_secondary_fitness);
      tertiary_fitness_history.push_back(curr_sol_tertiary_fitness);
    } else if (best_sol_cost == curr_sol_cost) {
      if (curr_sol_secondary_fitness > best_sol_secondary_fitness) {
        RCLCPP_INFO(logger,
                    "New Best Solution Found, Prev Cost: %lf, Cost: %lf",
                    best_cost, curr_sol_cost);
        best_sol = curr_sol;
        best_cost = curr_sol_cost;
        primary_fitness_history.push_back(curr_sol_cost);
        secondary_fitness_history.push_back(curr_sol_secondary_fitness);
        tertiary_fitness_history.push_back(curr_sol_tertiary_fitness);
      } else if (curr_sol_tertiary_fitness == best_sol_tertiary_fitness) {
        if (curr_sol_tertiary_fitness < best_sol_tertiary_fitness) {
          best_sol = curr_sol;
          best_cost = curr_sol_cost;
          primary_fitness_history.push_back(curr_sol_cost);
          secondary_fitness_history.push_back(curr_sol_secondary_fitness);
          tertiary_fitness_history.push_back(curr_sol_tertiary_fitness);
        }
      }
    }
  }
  inline bool check_solutions_SA(const double prev_fit, const double curr_fit,
                                 const double temp, bool primary = true) {
    std::uniform_real_distribution<double> unif(0, 1);
    double probability_threshold;
    double delta_E = curr_fit - prev_fit;
    if (std::abs(temp) < std::numeric_limits<double>::epsilon()) {
      probability_threshold = 0.0;
    } else {
      if (primary) {
        delta_E = -delta_E;
        probability_threshold = exp((delta_E) / temp);
      } else {
        probability_threshold = exp((delta_E) / temp);
      }
    }
    RCLCPP_INFO(logger,
                "Probability Threshold: %lf, prev_fit: %lf, curr_fit: %lf, "
                "delata_E: %lf, temp: %lf",
                probability_threshold, prev_fit, curr_fit, delta_E, temp);
    const double random_draw = unif(re);
    if (probability_threshold > random_draw) {
      RCLCPP_INFO(logger, "Solution Accepted");
      curr_sol_cost = curr_fit;
      return true;
    }
    return false;
  }
};

} // namespace ConnectedComm
#endif // Planner_HPP
