#! /usr/bin/env python3

"""Plot the power consumption and battery levels."""

import threading
from typing import Any, Optional

import matplotlib.animation as anim
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy.typing as npt
import rclpy
from cc_msgs.msg import BatteryLevel, ChargeLevel
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription


class PlotterNode(Node):
    """Node to graph the peak power consumption and battery levels.

    Attributes:
        sub_: Subscription to the peak power topic with the following msg format
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
        c_x: List of power consumption through time.
        y: List of time_steps through time.
        power_consumption: List of total power consumption through time.
        time_horizon: List of each time_step through time.
        _lock: threading lock for plotting the power consumption.
        _lock1: threading lock for plotting the battery levels.
        _fig: Figure for plotting the power consumption.
        _ax: Axis for plotting the power consumption.
        _fig_1: Figure for plotting the battery levels.
        _ax_1: Axis for plotting the battery levels.

    """

    def __init__(self) -> None:
        """Initialize."""
        super().__init__("plotter")
        self.charge_sub: Subscription = self.create_subscription(
            ChargeLevel, "charge_level", self.charge_graph_cb, 10
        )
        self.battery_socs: list[list[float]] = []
        self.battery_power_levels: list[list[float]] = []
        self.external_signal: list[float] = []
        self.resulting_signal: list[float] = []
        self.history_length: list[int] = []
        self.primary_fit_history: list[float] = []
        self.secondary_fit_history: list[float] = []
        self.times: list[float] = []
        self.min_time: int = 0
        self.max_time: int = 1339
        self._charge_lock = threading.Lock()
        self.charge_fig: plt.Figure
        self.charge_ax: npt.NDArray[Any]
        self.charge_fig, self.charge_ax = plt.subplots(3)
        self.charge_fig1, self.charge_ax1 = plt.subplots(3)
        self.declare_parameter("time_horizon", 1400)
        self.time_horizon: int = self.get_parameter("time_horizon").value
        plt.style.use("Solarize_Light2")
        self.count = 0

    def charge_graph_cb(self, msg: ChargeLevel) -> None:
        """Take the message and save off the data to be plotted."""
        with self._charge_lock:
            if len(self.history_length) != len(msg.primary_fitness_history):
                self.history_length = [
                    i for i in range(len(msg.primary_fitness_history))
                ]
            self.primary_fit_history = list(msg.primary_fitness_history)
            self.secondary_fit_history = list(msg.secondary_fitness_history)
            self.tertiary_fit_history = list(msg.tertiary_fitness_history)
            self.min_time = int(msg.min_time_idx)
            self.max_time = int(msg.max_time_idx)
            self.times = list(msg.time_idxs)
            bat_levels: list[BatteryLevel] = list(msg.charge_levels)
            self.battery_socs = [list(bat.charge_level) for bat in bat_levels]
            self.battery_power_levels = [
                list(bat.power_level) for bat in bat_levels]
            self.external_signal = list(msg.external_signal)
            self.resulting_signal = list(msg.resulting_signal)
            max_ = self.resulting_signal[self.max_time]
            if self.count == 0:
                self.get_logger().info(f"Max: {max_} at {self.max_time}")
            self.count += 1

    def plt_(self, _: Any) -> Any:
        """Plot the power consumption through time."""
        with self._charge_lock:
            if len(self.times) != 0:
                # clear the axes
                self.charge_ax[0].clear()
                self.charge_ax[1].clear()
                self.charge_ax[2].clear()
                # set ticks to 100
                self.charge_ax[0].xaxis.set_major_locator(
                    ticker.MultipleLocator(100))
                self.charge_ax[1].xaxis.set_major_locator(
                    ticker.MultipleLocator(100))
                self.charge_ax[2].xaxis.set_major_locator(
                    ticker.MultipleLocator(100))
                self.charge_ax[2].set_xlabel("Time Horizon (s)")
                self.charge_ax[0].set_ylabel("Power Level (kW)")
                self.charge_ax[0].set_title(
                    "External Signal and Power Profiles for all batteries."
                )
                self.charge_ax[0].plot(
                    self.times, self.external_signal, label="External Signal"
                )
                self.charge_ax[0].set_xlim(0, self.time_horizon)
                if len(self.battery_power_levels) > 0:
                    for i, power_level in enumerate(self.battery_power_levels):
                        self.charge_ax[0].plot(
                            self.times, power_level, label=f"Power Level {i}"
                        )
                self.charge_ax[0].legend()
                self.charge_ax[1].set_ylabel("Charge Level (kWh)")
                self.charge_ax[1].set_title(
                    "State of Charge for all batteries.")
                self.charge_ax[1].set_xlim(0, self.time_horizon)
                if len(self.battery_socs) > 0:
                    for i, power_level in enumerate(self.battery_socs):
                        self.charge_ax[1].plot(
                            self.times, power_level, label=f"Charge Level {i}"
                        )
                self.charge_ax[1].legend()
                self.charge_ax[2].set_title("Resulting Power Profile")
                self.charge_ax[2].set_ylabel("Power Level (kW)")
                self.charge_ax[2].set_xlim(0, self.time_horizon)
                self.charge_ax[2].plot(self.times, self.resulting_signal)
                return self.charge_ax
            else:
                return self.charge_ax

    def plt_1(self, _: Any) -> Any:
        """Plot convegence of the all fitness functions."""
        with self._charge_lock:
            if len(self.history_length) != 0:
                self.charge_ax1[0].clear()
                self.charge_ax1[1].clear()
                self.charge_ax1[2].clear()
                self.charge_ax1[0].set_title("Primary Fitness History")
                self.charge_ax1[0].set_ylabel("Max Power")
                self.charge_ax1[1].set_title("Secondary Fitness History")
                self.charge_ax1[1].set_ylabel("Max Min Difference")
                self.charge_ax1[2].set_title("Tertiary Fitness History")
                self.charge_ax1[2].set_xlabel("# of best solutions")
                self.charge_ax1[2].set_ylabel("Average SOC")
                self.charge_ax1[0].plot(
                    self.history_length, self.primary_fit_history, color="red"
                )
                self.charge_ax1[1].plot(
                    self.history_length, self.secondary_fit_history, color="green"
                )
                self.charge_ax1[2].plot(
                    self.history_length, self.tertiary_fit_history, color="purple"
                )
                return self.charge_ax1
            else:
                return self.charge_ax1

    def anim_plt(self) -> None:
        """Animate the charge consumption graph."""
        self.ani_charge = anim.FuncAnimation(
            self.charge_fig, self.plt_, interval=1000)

    def anim_plt1(self) -> None:
        """Animate the fitness graph."""
        self.ani_charge1 = anim.FuncAnimation(
            self.charge_fig1, self.plt_1, interval=1001
        )


def main(args: Optional[list[str]] = None) -> None:
    """Run node."""
    rclpy.init(args=args)
    node = PlotterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node.anim_plt()
    node.anim_plt1()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
