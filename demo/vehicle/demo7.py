# SM: Noting some things about the example

import timeit
from verse.agents.example_agent import CarAgent, NPCAgent
from verse.map.example_map import SimpleMap4
from verse import Scenario
from verse.plotter.plotter2D import *

from enum import Enum, auto
import plotly.graph_objects as go

class LaneObjectMode(Enum):
    Vehicle = auto()
    Ped = auto()        # Pedestrians
    Sign = auto()       # Signs, stop signs, merge, yield etc.
    Signal = auto()     # Traffic lights
    Obstacle = auto()   # Static (to road/lane) obstacles


class VehicleMode(Enum):
    Normal = auto()
    SwitchLeft = auto()
    SwitchRight = auto()
    Brake = auto()


class LaneMode(Enum):
    Lane0 = auto()
    Lane1 = auto()
    Lane2 = auto()
    Lane3 = auto()


class State:
    x = 0.0
    y = 0.0
    theta = 0.0
    v = 0.0
    vehicle_mode: VehicleMode = VehicleMode.Normal
    lane_mode: LaneMode = LaneMode.Lane0
    type: LaneObjectMode = LaneObjectMode.Vehicle

    def __init__(self, x, y, theta, v, vehicle_mode: VehicleMode, lane_mode: LaneMode, type: LaneObjectMode):
        pass


if __name__ == "__main__":
    input_code_name = './demo/vehicle/controller/example_controller8.py'
    scenario = Scenario()

    car = CarAgent('car1', file_name=input_code_name)
    scenario.add_agent(car)
    car = NPCAgent('car2')
    scenario.add_agent(car)
    car = CarAgent('car3', file_name=input_code_name)
    scenario.add_agent(car)
    car = NPCAgent('car4')
    scenario.add_agent(car)
    car = NPCAgent('car5')
    scenario.add_agent(car)
    car = CarAgent('car8', file_name=input_code_name)
    scenario.add_agent(car)
    car = NPCAgent('car6')
    scenario.add_agent(car)
    car = NPCAgent('car7')
    scenario.add_agent(car)
    tmp_map = SimpleMap4()
    scenario.set_map(tmp_map)
    scenario.set_init(
        [
            [[0, 0.0, 0, 1.0], [0.0, 0.0, 0, 1.0]],
            [[10, 0, 0, 0.5], [10, 0, 0, 0.5]],
            [[14, 3, 0, 0.6], [14, 3, 0, 0.6]],
            [[20, 3, 0, 0.5], [20, 3, 0, 0.5]],
            [[30, 0, 0, 0.5], [30, 0, 0, 0.5]],
            [[23, -3, 0, 0.6], [23, -3, 0, 0.6]],
            [[28.5, -3, 0, 0.5], [28.5, -3, 0, 0.5]],
            [[40, -6, 0, 0.5], [40, -6, 0, 0.5]],
        ],
        [
            (VehicleMode.Normal, LaneMode.Lane1),
            (VehicleMode.Normal, LaneMode.Lane1),
            (VehicleMode.Normal, LaneMode.Lane0),
            (VehicleMode.Normal, LaneMode.Lane0),
            (VehicleMode.Normal, LaneMode.Lane1),
            (VehicleMode.Normal, LaneMode.Lane2),
            (VehicleMode.Normal, LaneMode.Lane2),
            (VehicleMode.Normal, LaneMode.Lane3),
        ],
        [
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
            (LaneObjectMode.Vehicle,),
        ],

    )
    import timeit
    time = timeit.default_timer()
    traces = scenario.simulate(60, 0.05)
    print("\x1b[31mrun1\x1b[0m", timeit.default_timer() - time)
    fig = go.Figure()
    fig = simulation_tree(traces, tmp_map, fig, 1,
                          2, 'lines', 'trace', print_dim_list=[1, 2])
    fig.show()

    time = timeit.default_timer()
    traces = scenario.simulate(60, 0.05)
    print("\x1b[31mrun2\x1b[0m", timeit.default_timer() - time)
    fig = go.Figure()
    fig = simulation_tree(traces, tmp_map, fig, 1,
                          2, 'lines', 'trace', print_dim_list=[1, 2])
    fig.show()
