#include "power_energy_propogation/power_solution.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <iostream>

class TestBatteryAggregate : public ::testing::Test {
public:
  TestBatteryAggregate() {}
  ~TestBatteryAggregate() {}
};

TEST(TestBatteryAggregate, TestAddEvent) {
  int num_slots = 500;
  int num_bat = 1;
  float time_step = 1.;
  std::vector<double> external_load(num_slots, 0.);
  std::vector<double> batt_rate(num_bat, 10000.);
  std::vector<double> soc_max(num_bat, 100000.);
  std::vector<double> soc_min(num_bat, 0.);
  ConnectedComm::PowerSolution bat(num_slots, num_bat, time_step, external_load,
                                    soc_max, batt_rate, soc_min);
  double start_time = 0.01;
  double end_time = start_time;
  double activity = 1.;
  double rate = 1.;
  double battery_id = 0;

  for (int i = 0; i < num_slots - 1; ++i) {
    end_time += 1.;
    bat.add_event(start_time, end_time, activity, rate, battery_id);
  }
  ASSERT_EQ(bat.get_max_time_slot_idx(), 0);
  ASSERT_EQ(bat.get_min_time_slot_idx(), 499);
}

TEST(TestBatteryAggregate, TestChargePropogation) {
  int num_slots = 500;
  int num_bat = 1;
  float time_step = 1.;
  float soc_start = 500;
  std::vector<double> external_load(num_slots, 0.);
  std::vector<double> batt_rate(num_bat, 100.);
  std::vector<double> soc_max(num_bat, 100000.);
  std::vector<double> soc_min(num_bat, 0.);
  ConnectedComm::PowerSolution bat(num_slots, num_bat, time_step, external_load,
                                    soc_max, batt_rate, soc_min, soc_start);
  double start_time = 0.01;
  double end_time = 6*60 + .01;
  double activity = 1.;
  double rate = -1.;
  double battery_id = 0;
  bat.add_event(start_time, end_time, activity, rate, battery_id);
  bat.print_time_slots(0);
  double charge = bat.battery_id_to_energy_time_slots[0][499].energy;
  double charge_expect = 500 - ((6*60)/60);
  double tolerance = 5e-2;  // Adjust the tolerance as needed
  ASSERT_NEAR(charge, charge_expect, tolerance);
}
