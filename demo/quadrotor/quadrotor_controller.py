from enum import Enum, auto
import copy


class CraftMode(Enum):
    Follow_Waypoint = auto()


class State:
    x = 0.0
    y = 0.0
    z = 0.0
    vx = 0.0
    vy = 0.0
    vz = 0.0
    craft_mode: CraftMode = CraftMode.Follow_Waypoint
    waypoint_index: int = 0
    done_flag = 0.0  # indicate if the quad rotor reach the waypoint

    def __init__(self, x, y, z, vx, vy, vz, waypoint_index, done_flag, craft_mode):
        pass


def controller(ego: State):
    output = copy.deepcopy(ego)
    if ego.done_flag > 0:
        output.done_flag = 0
        if ego.waypoint_index == 0:
            output.waypoint_index = 1
        if ego.waypoint_index == 1:
            output.waypoint_index = 4
        if ego.waypoint_index == 1:
            output.waypoint_index = 2
        if ego.waypoint_index == 2:
            output.waypoint_index = 3
        if ego.waypoint_index == 4:
            output.waypoint_index = 5

    return output
