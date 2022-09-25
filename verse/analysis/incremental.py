from collections import defaultdict
from dataclasses import dataclass
from pprint import pp
from typing import DefaultDict, List, Tuple, Optional, Dict
from verse.analysis import AnalysisTreeNode
from intervaltree import IntervalTree

from verse.analysis.dryvr import _EPSILON
from verse.parser.parser import ControllerIR, ModePath

@dataclass
class CachedTransition:
    inits: Dict[str, List[float]]
    transition: int
    disc: List[str]
    cont: List[float]
    paths: List[ModePath]

@dataclass
class CachedSegment:
    trace: List[List[float]]
    asserts: List[str]
    transitions: List[CachedTransition]
    controller: ControllerIR
    run_num: int
    node_id: int

def convert_transitions(agent_id, transit_agents, inits, transition, trans_ind):
    if agent_id in transit_agents:
        return [CachedTransition(inits, trans_ind, mode, init, paths) for _id, mode, init, paths in transition]
    else:
        return []

@dataclass
class CachedTube:
    tube: List[List[List[float]]]

    def __eq__(self, other) -> bool:
        if other is None:
            return False
        return (self.tube == other.tube).any()

class SimTraceCache:
    def __init__(self):
        self.cache: DefaultDict[tuple, IntervalTree] = defaultdict(IntervalTree)

    def add_segment(self, agent_id: str, node: AnalysisTreeNode, transit_agents: List[str], trace: List[List[float]], transition, trans_ind: int, run_num: int):
        key = (agent_id,) + tuple(node.mode[agent_id])
        init = node.init[agent_id]
        tree = self.cache[key]
        assert_hits = node.assert_hits or {}
        # pp(('add seg', agent_id, *node.mode[agent_id], *init))
        for i, val in enumerate(init):
            if i == len(init) - 1:
                transitions = convert_transitions(agent_id, transit_agents, node.init, transition, trans_ind)
                entry = CachedSegment(trace, assert_hits.get(agent_id), transitions, node.agent[agent_id].controller, run_num, node.id)
                tree[val - _EPSILON:val + _EPSILON] = entry
                return entry
            else:
                next_level_tree = IntervalTree()
                tree[val - _EPSILON:val + _EPSILON] = next_level_tree
                tree = next_level_tree
        raise Exception("???")

    @staticmethod
    def iter_tree(tree, depth: int) -> List[List[float]]:
        if depth == 0:
            return [[(i.begin + i.end) / 2, (i.data.run_num, i.data.node_id, [t.transition for t in i.data.transitions])] for i in tree]
        res = []
        for i in tree:
            mid = (i.begin + i.end) / 2
            subs = SimTraceCache.iter_tree(i.data, depth - 1)
            res.extend([mid] + sub for sub in subs)
        return res

    def get_cached_inits(self, n: int):
        inits = defaultdict(list)
        for key, tree in self.cache.items():
            inits[key[0]].extend((*key[1:], *init) for init in self.iter_tree(tree, n))
        inits = dict(inits)
        return inits

    def check_hit(self, agent_id: str, mode: Tuple[str], init: List[float]) -> Optional[CachedSegment]:
        key = (agent_id,) + tuple(mode)
        if key not in self.cache:
            return None
        tree = self.cache[key]
        for cont in init:
            next_level_entries = list(tree[cont])
            if len(next_level_entries) == 0:
                return None
            tree = min(next_level_entries, key=lambda e: (e.end + e.begin) / 2 - cont).data
        assert isinstance(tree, CachedSegment)
        return tree
    
class ReachTraceCache:
    def __init__(self):
        self.cache: DefaultDict[tuple, IntervalTree] = defaultdict(IntervalTree)

    def add_tube(self, agent_id: str, mode: Tuple[str], init: List[List[float]], trace: List[List[List[float]]]):
        key = (agent_id,) + tuple(mode)
        init = list(map(list, zip(*init)))
        tree = self.cache[key]
        for i, (low, high) in enumerate(init):
            if i == len(init) - 1:
                entry = CachedTube(trace)
                tree[low:high + _EPSILON] = entry
                return entry
            else:
                next_level_tree = IntervalTree()
                tree[low:high + _EPSILON] = next_level_tree
                tree = next_level_tree
        raise Exception("???")

    def check_hit(self, agent_id: str, mode: Tuple[str], init: List[List[float]]) -> Optional[CachedTube]:
        key = (agent_id,) + tuple(mode)
        if key not in self.cache:
            return None
        tree = self.cache[key]
        for low, high in list(map(list, zip(*init))):
            next_level_entries = [t for t in tree[low:high + _EPSILON] if t.begin <= low and high <= t.end]
            if len(next_level_entries) == 0:
                return None
            tree = min(next_level_entries, key=lambda e: low - e.begin + e.end - high).data
        assert isinstance(tree, CachedTube)
        return tree
