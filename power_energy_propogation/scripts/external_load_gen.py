#!/usr/bin/env python3

"""Generates an external uncontrolled load for the simulation."""
import datetime as dt
import os
from typing import Optional

import numpy as np
import numpy.typing as npt
import rclpy
from ament_index_python.packages import get_package_share_directory
from cc_msgs.srv import LoadProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile


class ExternalLoadGenerator(Node):
    """Node that creates the uncontrolled load.

    Attributes:
        max_power: The maximum allowed power rate for the exterenal signal
        min_power: The minimum allowed power rate for the external signal

    """

    def __init__(self) -> None:
        """Initialize."""
        super().__init__("external_load_generator")
        self.declare_parameter("soc_max", 1000.0)
        self.declare_parameter("soc_min", 50.0)
        self.declare_parameter("power_filename", "power_15.csv")
        self.declare_parameter("solar_filename", "dc_average_by_time.csv")
        self.declare_parameter("nsteps", 1000)
        self.declare_parameter("timestep", 1.0)
        self.soc_max: float = self.get_parameter("soc_max").value
        self.soc_min: float = self.get_parameter("soc_min").value
        self.power_filename: str = self.get_parameter("power_filename").value
        self.solar_filename: str = self.get_parameter("solar_filename").value
        self.nsteps: int = self.get_parameter("nsteps").value
        self.timestep: float = self.get_parameter("timestep").value * 60
        qos_profile = QoSProfile(depth=200)
        self.load_service = self.create_service(
            LoadProfile, "load_profile", self.generate_load_cb, qos_profile=qos_profile
        )

    def generate_load_cb(
        self, req: LoadProfile.Request, res: LoadProfile.Response
    ) -> LoadProfile.Response:
        """Return response from load request.

        Args:
            req: The request with a form of (float64 timestep, int32 nsteps)
            res: The response with the form (float64[] load_profile)

        """
        self.get_logger().info("Recieved req")
        self.get_logger().info(
            f"Generating load profile with timestep: {float(req.timestep)} and"
            f" nstep: {int(req.nsteps)}"
        )
        load_profile = []
        load_profile = self.generate_power_from_data()
        self.get_logger().info(f"Generated Load...")
        res.load_profile = load_profile
        self.get_logger().info(f"Returning load with size: {len(res.load_profile)}")
        return res

    def produce_sinusoidal_brownian(
        self,
        tau: float,
        tvec: list[float],
        p0: float = 0.0,
    ) -> list[float]:
        """Produce brownian motion with a sinusoidal undercurrent.

        Args:
            tau: the time constant for convergence on the brownian motion
            tvec: time values of interest
            p0: initial power level

        Returns:
            The power over time

        """
        freq = 1 / 350  # frequency (1/sec) of the sinusoid
        amp = 200  # amplitude of the sinuoisoid
        q = 10  # the variance of the white noise
        b = 0.0  # the brownian motion
        p_vec: list[float] = [p0]
        for k in range(1, len(tvec)):
            # Calculate the brownian motion
            dt = tvec[k] - tvec[k - 1]
            b = b + dt * (-1.0 / tau * b + np.random.normal(0.0, q))
            # Store the updated power
            p_vec.append(np.abs(np.sin(freq * tvec[k]) * amp + p0 + b))
        return p_vec

    def generate_power_from_data(self) -> list[float]:
        """Generate power usage and solar production from csv files.

        Returns:
            list[float]: Overall power consumption with the appropriate time
            discretization.

        """
        # Load data ---
        power_data_path = os.path.join(
            get_package_share_directory("power_energy_propogation"),
            "data",
            self.power_filename,
        )
        solar_data_path = os.path.join(
            get_package_share_directory("power_energy_propogation"),
            "data",
            self.solar_filename,
        )
        # get toady's date
        today = dt.datetime.today().date()
        # create callable that prefills the date so that it can be passed to
        # the np genfromtxt function

        def _convert(x: bytes) -> dt.datetime:
            return _convert_timestamp(x, today)
        # Load data convert timestamps to datetime objects with nested def
        self.get_logger().info(f"Gen from text")
        power_data = np.genfromtxt(
            power_data_path,
            delimiter=",",
            converters={0: _convert},
            names=True,
            dtype=None,
        )
        solar_data = np.genfromtxt(
            solar_data_path,
            delimiter=",",
            names=True,
        )
        self.get_logger().info(f" After Gen from text")
        # subtract solar from total power to get power consumption
        total_power_draw = power_data["DC_POWER"] - solar_data["DC_POWER"]
        # get today's date
        today = dt.datetime.today().date()
        # get the min of the date
        today_min = dt.datetime.combine(today, dt.time.min)

        # generate the datetimes of interest
        times_of_intrest = np.array(
            [i * self.timestep for i in range(self.nsteps)])
        # convert datetimes to seconds since midnight
        power_times: npt.NDArray[np.float64] = np.array(
            [
                convert_datetime_to_seconds_since_midnight(
                    curr_datetime, today, today_min
                )
                for curr_datetime in power_data["DATE_TIME"]
            ]
        )
        self.get_logger().info(f"times of intrest: {times_of_intrest[:25]}")
        self.get_logger().info(f"power times: {power_times}")
        # interpolate the power data at the times of intrest
        interp: npt.NDArray[np.float64] = np.interp(
            times_of_intrest, power_times, total_power_draw
        )
        # add noise
        interp_list: list[float] = interp.tolist()
        return interp_list


def _convert_timestamp(x: bytes, today: dt.date) -> dt.datetime:
    """Convert timestamp to datetime."""
    str_ = x if isinstance(x, str) else x.decode("utf-8")
    modified = dt.datetime.strptime(str_, "%H:%M")
    return dt.datetime.combine(today, modified.time())


def convert_datetime_to_seconds_since_midnight(
    x: dt.datetime, today: dt.date, today_min: dt.datetime
) -> float:
    """Convert datetime to seconds since midnight.

    Args:
        x: the datetime which has the wrong date
        today: the date to use
        today_min: the datetime of midnight

    Returns:
        float: seconds since midnight
    """
    info = dt.datetime.combine(today, x.time())
    return float((info - today_min).total_seconds())


def main(args: Optional[list[str]] = None) -> None:
    """Init main."""
    try:
        rclpy.init(args=args)
        exec = MultiThreadedExecutor()
        node = ExternalLoadGenerator()
        exec.add_node(node)
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down external load generator")
        node.destroy_node()
        exec.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
