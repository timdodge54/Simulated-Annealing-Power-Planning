"""This is a test launch file."""
from launch_ros.actions import Node  # type: ignore
from launch import LaunchDescription  # type: ignore


def generate_launch_description() -> LaunchDescription:
    """Generate launch description with multiple components."""
    # Simulation parameters ----------------
    battery_capacity = 100.0
    battery_rate = 50.0
    num_batteries = 3
    num_add_events = 1000
    battery_capacaties = [300.0 for _ in range(num_batteries)]
    battery_minimums = [0.0 for _ in range(num_batteries)]
    battery_rates = [50.0 for _ in range(num_batteries)]
    time_horizon = 1400
    time_step = 1.0
    temp_max = 5.0
    temp_min = 1.0
    fitness_scalar = 1000.0
    temp_cooling_rate = .7
    planner_type = "SA"
    soc_max = 100.0
    soc_min = 0.0
    starting_soc = 200.0
    # planner_type = "GREEDY"
    number_of_planners = 10
    ld = LaunchDescription()
    # --------------------------------------
    power_grapher = Node(
        package="power_energy_propogation",
        executable="plotter_node.py",
        name="grapher",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "time_horizon": time_horizon,
            }
        ],
    )

    for i in range(number_of_planners):
        ld.add_action(Node(
            package="power_energy_propogation",
            executable="power_node",
            parameters=[
                {
                    "time_horizon": time_horizon,
                    "time_step": time_step,
                    "battery_capacity": battery_capacity,
                    "battery_rate": battery_rate,
                    "battery_rates": battery_rates,
                    "num_batteries": num_batteries,
                    "num_add_events": num_add_events,
                    "battery_capacaties": battery_capacaties,
                    "battery_minimums": battery_minimums,
                    "temp": temp_max,
                    "planner_type": planner_type,
                    "temp_cooling_rate": temp_cooling_rate,
                    "soc_start": starting_soc,
                    "temp_min": temp_min,
                    "fitness_scalar": fitness_scalar,
                    "planner_number": i,
                }
            ],
            remappings=[("/charge_level", f"/charge_level_{i + 1}")],
            name=f"power_node_{i + 1}",
            output="screen",
            emulate_tty=True,
            # arguments = ["--ros-args", "--log-level", "debug"],
        ))

    external_generation_node = Node(
        package="power_energy_propogation",
        executable="external_load_gen.py",
        parameters=[
            {
                "soc_min": soc_min,
                "soc_max": soc_max,
                "power_filename": "august_power_avg_15.csv",
                "solar_filename": "dc_average_by_time.csv",
                "nsteps": time_horizon,
                "timestep": time_step,
            }
        ],
        name="external_load_gen",
        output="screen",
        emulate_tty=True,
    )

    filter = Node(
        package="power_energy_propogation",
        executable="filter_solutions_node.py",
        parameters=[{
                "number_of_planners": number_of_planners,
        }],
        name="filter_node",
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(filter)
    ld.add_action(external_generation_node)
    ld.add_action(power_grapher)
    return ld
