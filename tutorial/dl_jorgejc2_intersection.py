from enum import Enum, auto


class AgentMode(Enum):
    Normal = auto()
    SwitchLeft = auto()
    SwitchRight = auto()
    Brake = auto()
    Accel = auto()

class TrackMode(Enum):
    T0 = auto()
    T1 = auto()


class State:
    x: float
    y: float
    theta: float
    v: float
    agent_mode: AgentMode
    track_mode: TrackMode

    def __init__(self, x, y, theta, v, agent_mode: AgentMode, track_mode: TrackMode):
        pass


def vehicle_front(ego, others, track_map):
    res = any(
        (
            6
            < track_map.get_longitudinal_position(other.track_mode, [other.x, other.y])
            - track_map.get_longitudinal_position(ego.track_mode, [ego.x, ego.y])
            < 8
        )
        for other in others
    )
    return res


def vehicle_close(ego, others):
    return any(-1 < ego.x - other.x < 1 and -1 < ego.y - other.y < 1 for other in others)


from typing import List
import copy


def decisionLogic(ego: State, others: List[State], track_map):
    output = copy.deepcopy(ego)
    if ego.agent_mode == AgentMode.Normal:

        # add new case which hits the breaks
        if vehicle_front(ego, others, track_map):
            if track_map.h_exist(ego.track_mode, ego.agent_mode, AgentMode.Brake):
                output.agent_mode = AgentMode.Brake
                output.track_mode = track_map.h(ego.track_mode, ego.agent_mode, AgentMode.Brake)

    # lat_dist = track_map.get_lateral_distance(ego.track_mode, [ego.x, ego.y])
    # if ego.agent_mode == AgentMode.SwitchLeft:
    #     if lat_dist >= 2.5:
    #         output.agent_mode = AgentMode.Normal
    #         output.track_mode = track_map.h(ego.track_mode, ego.agent_mode, AgentMode.Normal)
    # if ego.agent_mode == AgentMode.SwitchRight:
    #     if lat_dist <= -2.5:
    #         output.agent_mode = AgentMode.Normal
    #         output.track_mode = track_map.h(ego.track_mode, ego.agent_mode, AgentMode.Normal)

    if ego.agent_mode == AgentMode.Brake:
        if not vehicle_front(ego, others, track_map):
            output.agent_mode = AgentMode.Accel
            output.track_mode = track_map.h(ego.track_mode, ego.agent_mode, AgentMode.Accel)

    # if ego.agent_mode == AgentMode.Accel:


    assert not vehicle_close(ego, others)
    return output
