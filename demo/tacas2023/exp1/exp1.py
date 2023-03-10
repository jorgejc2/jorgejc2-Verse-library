from quadrotor_agent import QuadrotorAgent
from verse import Scenario
from verse.plotter.plotter3D_new import *
from verse.plotter.plotter3D import *
from verse.map.example_map.map_tacas import M6 
from verse.scenario.scenario import Benchmark
from enum import Enum, auto
import warnings

warnings.filterwarnings("ignore")

class TacticalMode(Enum):
    Normal = auto()
    MoveUp = auto()
    MoveDown = auto()


class TrackMode(Enum):
    T0 = auto()
    T1 = auto()
    T2 = auto()
    M01 = auto()
    M10 = auto()
    M12 = auto()
    M21 = auto()


if __name__ == "__main__":
    import sys
    input_code_name = './demo/tacas2023/exp1/quadrotor_controller3.py'

    bench = Benchmark(sys.argv)
    time_step = 0.1
    quadrotor1 = QuadrotorAgent(
        'test1', file_name=input_code_name, t_v_pair=(1, 1), box_side=[0.4]*3)
    init_l_1 = [9.5, 0, -0.35, 0, 0, 0]
    init_u_1 = [10.2, 0.7, 0.35, 0, 0, 0]
    quadrotor1.set_initial(
        [init_l_1, init_u_1],
        (TacticalMode.Normal, TrackMode.T1)
    )
    bench.scenario.add_agent(quadrotor1)

    quadrotor2 = QuadrotorAgent(
        'test2', file_name=input_code_name, t_v_pair=(1, 0.3), box_side=[0.4]*3)
    init_l_2 = [3, 9, -0.35, 0, 0, 0]
    init_u_2 = [3.7, 9.7, 0.35, 0, 0, 0]
    quadrotor2.set_initial(
        [init_l_2, init_u_2],
        (TacticalMode.Normal, TrackMode.T1)
    )
    bench.scenario.add_agent(quadrotor2)

    tmp_map = M6()
    bench.scenario.set_map(tmp_map)
    # scenario.set_sensor(QuadrotorSensor())
    if not bench.config.compare:
        traces = bench.run(40, time_step)
        if bench.config.plot:
            import pyvista as pv
            fig = pv.Plotter()
            fig = plot3dMap(tmp_map, ax=fig, width=0.05)
            fig = plot3dReachtube(traces, 'test1',1,2,3,'r',fig, edge = True)
            fig = plot3dReachtube(traces, 'test2',1,2,3,'b',fig, edge = True)
            fig.set_background('#e0e0e0')
            fig.show()
        if bench.config.dump:
            traces.dump('demo/tacas2023/exp1/output1.json')
        bench.report()
    else:
        traces1, traces2 = bench.compare_run(40, time_step)
    
