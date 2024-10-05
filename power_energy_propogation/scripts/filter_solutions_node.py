#!/usr/bin/env python3
"""Node for filter solutions to find the best."""

import threading
import typing

import rclpy
from cc_msgs.msg import BatteryLevel, ChargeLevel
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription


class FilterSolutionsNode(Node):
    """Node takes all recieved solutions find the best one and pass it one.

    Attributes:
        num_planners: number of planners to recieve plans from
        recieved_plans: list of bools to track which plans have been recieved
        count_received: number of plans recieved
        cbg_publish: callback group for all callback functions
        subs: list of subscribers with the topic name of
            "charge_level_<planner_number>" with msg format of the following
            -----------------------------------
            BatteryLevel[] charge_levels // list of battery soc and power level
            float64[] external_signal // list of external power signal
            float64[] resulting_signal // list of resulting power signal
            float64[] primary_fitness_history
            float64[] secondary_fitness_history
            float64[] tertiary_fitness_history
            int32[] time_idxs // list of times in minutes
            int32 min_time_idx // min time index
            int32 max_time_idx // max time index
            int32 planner_num // planner number
            -----------------------------------
        pub: publisher that publishes the best plan with the same msg format as
            above
        primary_fit_history: list of primary fitness history for each planner
        secondary_fit_history: list of secondary fitness history for each
            planner
        tertiary_fit_history: list of tertiary fitness history for each planner
        battery_socs: list of battery socs for each planner
        resulting_signal: list of resulting signals for each planner
        external_signal: list of the external signal
        times: list of times in minutes
        max_times: list of max times for each planner
        min_times: list of min times for each planner
        lock: lock for thread safety
        published_plan: bool to track if a plan has been published
        timer: timer to publish the best plan
    """

    def __init__(self) -> None:
        """Initialilze."""
        super().__init__("filter_remap_node")
        # get the number of planners from the launch file
        self.declare_parameter("number_of_planners", 1)
        self.num_planners: int = self.get_parameter("number_of_planners").value
        # set the recived plans all to false
        self.recieved_plans: list[bool] = [
            False for _ in range(self.num_planners)]
        # initalize the count recieved to 0
        self.count_received: int = 0
        # set the callback group
        self.cbg_publish = MutuallyExclusiveCallbackGroup()
        # loop through the number of planners and create subscriptions
        # with name "charge_level_i+1" to conform with ros naming
        self.subs: list[Subscription] = [
            self.create_subscription(
                ChargeLevel,
                f"charge_level_{i + 1}",
                self.get_plan_cb,
                10,
                callback_group=self.cbg_publish,
            )
            for i in range(self.num_planners)
        ]
        # create the publisher that will send to the plotter
        self.pub: Publisher = self.create_publisher(
            ChargeLevel, "charge_level", 10)
        # initialize all lists to conform with the number of planners
        self.primary_fit_history: list[list[float]] = [
            [0.0] for _ in range(self.num_planners)
        ]
        self.secondary_fit_history: list[list[float]] = [
            [0.0] for _ in range(self.num_planners)
        ]
        self.tertiary_fit_history: list[list[float]] = [
            [0.0] for _ in range(self.num_planners)
        ]
        self.battery_socs: list[list[float]] = [
            [0.0] for _ in range(self.num_planners)
        ]
        self.battery_power_levels: list[list[float]] = [
            [0.0] for _ in range(self.num_planners)
        ]
        self.resulting_signal: list[list[float]] = [
            [0.0] for _ in range(self.num_planners)
        ]
        self.charge_levels: list[list[BatteryLevel]] = [
            [] for _ in range(self.num_planners)
        ]
        self.min_times: list[int] = [-1 for _ in range(self.num_planners)]
        self.max_times: list[int] = [-1 for _ in range(self.num_planners)]
        self.external_signal: list[float] = []
        self._lock: threading.Lock = threading.Lock()
        self.times: list[int] = []
        self.published_plan: bool = False
        # create a timer to publish plans
        self.timer = self.create_timer(
            15.0, self.publish_best_plan, callback_group=self.cbg_publish
        )

    def get_plan_cb(self, msg: ChargeLevel) -> None:
        """Save off plan when msg recieved."""
        if not self.recieved_plans[msg.planner_num]:
            self.count_received += 1
            self.get_logger().info(f"Proccessing planner {msg.planner_num}")
            plan_num: int = msg.planner_num
            self.primary_fit_history[plan_num] = list(
                msg.primary_fitness_history)
            self.secondary_fit_history[plan_num] = list(
                msg.secondary_fitness_history)
            self.tertiary_fit_history[plan_num] = list(
                msg.tertiary_fitness_history)
            self.min_times[plan_num] = int(msg.min_time_idx)
            self.max_times[plan_num] = int(msg.max_time_idx)
            bat_levels: list[BatteryLevel] = list(msg.charge_levels)
            # ignoring because mypy cannont infer msg class type
            self.battery_socs[plan_num] = [
                list(bat.charge_level) for bat in bat_levels]  # type: ignore
            self.battery_power_levels[plan_num] = [
                list(bat.power_level) for bat in bat_levels  # type: ignore
            ]
            self.resulting_signal[plan_num] = list(msg.resulting_signal)
            self.charge_levels[plan_num] = list(msg.charge_levels)
            # only save of external signal and times once because it is
            # identical for all plans
            if len(self.external_signal) == 0:
                self.external_signal = list(msg.external_signal)
                self.times = list(msg.time_idxs)
            max_idx: int = self.max_times[plan_num]
            max_ = self.resulting_signal[plan_num][max_idx]
            self.get_logger().info(f"Max time: {max_idx} with value {max_}")
            self.recieved_plans[msg.planner_num] = True

    def publish_best_plan(self) -> None:
        """Looks through all plans and publishes the best one."""
        if self.count_received != self.num_planners:
            self.get_logger().error(
                f"all plans not recieved... Recieved: {self.count_received}"
            )
        best_plan_idx: int
        best_plan_prim_fitness: float
        best_plan_sec_fitness: float
        # loop through all plans
        for i in range(self.num_planners):
            # if the plann has been recieved
            if self.recieved_plans[i]:
                # if it is the first plan initialize all variables
                if i == 0:
                    best_plan_idx = i
                    best_plan_prim_fitness = self.primary_fit_history[i][-1]
                    best_plan_sec_fitness = self.secondary_fit_history[i][-1]
                    best_plan_tert_fitness = self.tertiary_fit_history[i][-1]
                else:
                    # otherwise greedily find the best plan utilizing the
                    # dominating objectives
                    self.get_logger().warn(f"Comparing best with plan {i}")
                    curr_prim_fitness: float = self.primary_fit_history[i][-1]
                    curr_sec_fitness: float = self.secondary_fit_history[i][-1]
                    curr_tert_fitness: float = self.tertiary_fit_history[i][-1]
                    self.get_logger().warn(
                        f"Comparing current {curr_prim_fitness} and"
                        f" {curr_sec_fitness} to best {best_plan_prim_fitness}"
                        f" and {best_plan_sec_fitness}"
                    )
                    if curr_prim_fitness < best_plan_prim_fitness:
                        best_plan_idx = i
                        best_plan_prim_fitness = curr_prim_fitness
                        best_plan_sec_fitness = curr_sec_fitness
                    elif curr_prim_fitness == best_plan_prim_fitness:
                        if curr_sec_fitness < best_plan_sec_fitness:
                            best_plan_idx = i
                            best_plan_prim_fitness = curr_prim_fitness
                            best_plan_sec_fitness = curr_sec_fitness
                        elif curr_sec_fitness == best_plan_sec_fitness:
                            if curr_tert_fitness > best_plan_tert_fitness:
                                best_plan_idx = i
                                best_plan_prim_fitness = curr_prim_fitness
                                best_plan_sec_fitness = curr_sec_fitness
                                best_plan_tert_fitness = curr_tert_fitness
        self.get_logger().info(
            f"Best plan: {best_plan_idx} with primary "
            f"fitness {best_plan_prim_fitness} and"
            f" secondary fitness {best_plan_sec_fitness}"
            f" tertiary fitness {best_plan_tert_fitness}"
        )
        # set the msg with the best plans values
        msg: ChargeLevel = ChargeLevel()
        msg.primary_fitness_history = self.primary_fit_history[best_plan_idx]
        msg.secondary_fitness_history = self.secondary_fit_history[best_plan_idx]
        msg.tertiary_fitness_history = self.tertiary_fit_history[best_plan_idx]
        msg.external_signal = self.external_signal
        msg.time_idxs = self.times
        msg.min_time_idx = self.min_times[best_plan_idx]
        msg.max_time_idx = self.max_times[best_plan_idx]
        msg.charge_levels = self.charge_levels[best_plan_idx]
        msg.resulting_signal = self.resulting_signal[best_plan_idx]
        self.get_logger().info(
            f" Primary length {len(self.primary_fit_history[best_plan_idx])}"
            f" Teritary length {len(self.tertiary_fit_history[best_plan_idx])}"
        )
        self.pub.publish(msg)


def main(args: typing.Optional[list[str]] = None) -> None:
    """Main function."""
    try:
        rclpy.init(args=args)
        node = FilterSolutionsNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
