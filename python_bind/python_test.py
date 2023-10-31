# Agents parameters
DIFF_DRIVE_V_MAX    = 'v_max'
DIFF_DRIVE_V_MIN    = 'v_min'
DIFF_DRIVE_W_MAX    = 'w_max'
DIFF_DRIVE_W_MIN    = 'w_min'
AGENT_SIZE          = 'size'
VISIBILITY_RADIUS   = 'r_vis'

# MPPI parameters

USE_CAR_LIKE_DYN    = "car_like_dyn"
SAMPLE_BATCH_SIZE   = 'batch_size'
SAMPLE_TIME_STEPS   = 'time_steps'
TIME_STEP           = 'dt'
INIT_VALUE          = 'init_value'
CONTROL_MEAN        = 'control_mean'
CONTROL_COV         = 'control_cov'
RUNNING_COST_WEIGH  = 'dist_to_goal_weight'
TERMINAL_WEIGHT     = 'terminal_weight'
SOFTMAX_TEMPERATURE = 'mppi_lambda'
MAX_SAFE_DIST_STEP  = 'max_safe_distr_step'
MAX_BVC_COST_STEP   = 'max_bvc_cost_step'
MA_COST_EN          = 'ma_cost_en'
IMPORTANCE_SAMPL_EN = 'importance_sampl_en'
BVC_WEIGHT          = 'bvc_weight'
MA_DIST_WEIGHT      = 'ma_dist_weight'
MA_COLLISION_WEIGHT = 'ma_collision_weight'
BVC_W_BASE          = 'bvc_w_base'
BVC_W_MEAN          = 'bvc_w_mean'
BVC_W_STD           = 'bvc_w_std'
MA_DIST_THREASHOLD  = 'ma_dist_threashhold'
MAX_NEIGHBORS_NUM   = 'max_neighbors_num'



import sys
from os.path import dirname

# MPPI library and python must be compiled. Below you need to specify the path to the python binding library file!
sys.path.append(dirname("../build/release/mppi_ca_cpp.cpython-311-x86_64-linux-gnu.so"))

import mppi_ca_cpp
import numpy as np
import timeit

agents_num = 5
max_sim_steps   = 500
dt              = 0.1
goal_xy_offset  = 0.3

default_r_vis   = 30.0
default_ag_size = 0.3
default_v_max   = 1.0
default_v_min   = -0.2
default_w_max   = np.pi/3
default_w_min   = -np.pi/3


alg_params = dict()
default_agent_params = dict()
exp_params = dict()

default_agent_params[VISIBILITY_RADIUS] = default_r_vis
default_agent_params[AGENT_SIZE] = default_ag_size
default_agent_params[DIFF_DRIVE_V_MAX] = default_v_max
default_agent_params[DIFF_DRIVE_V_MIN] = default_v_min
default_agent_params[DIFF_DRIVE_W_MAX] = default_w_max
default_agent_params[DIFF_DRIVE_W_MIN] = default_w_min


alg_params[SAMPLE_BATCH_SIZE]   = 700
alg_params[SAMPLE_TIME_STEPS]   = 25
alg_params[TIME_STEP]           = dt
alg_params[INIT_VALUE]          = np.array([0.0, 0.0])
alg_params[CONTROL_MEAN]        = np.array([0.0, 0.0])
alg_params[CONTROL_COV]         = np.array([0.05, 0.0, 0.0, 0.01], dtype=np.float64).reshape((2, 2))
alg_params[RUNNING_COST_WEIGH]  = 340.0
alg_params[TERMINAL_WEIGHT]     = 2500
alg_params[SOFTMAX_TEMPERATURE] = 2.25
alg_params[MAX_SAFE_DIST_STEP]  = 0
alg_params[MAX_BVC_COST_STEP]   = 0
alg_params[MA_COST_EN]          = True
alg_params[IMPORTANCE_SAMPL_EN] = False
alg_params[BVC_WEIGHT]          = 0
alg_params[MA_DIST_WEIGHT]      = 1480
alg_params[MA_COLLISION_WEIGHT] = 100000000.0
alg_params[BVC_W_BASE]          = 0.0
alg_params[BVC_W_MEAN]          = 0.0
alg_params[BVC_W_STD]           = 0.0
alg_params[MA_DIST_THREASHOLD]  = 4.0
alg_params[MAX_NEIGHBORS_NUM]   = 60
alg_params[USE_CAR_LIKE_DYN]    = True

agents_params = [default_agent_params] * agents_num


controllers = [mppi_ca_cpp.Controller() for a_id in range(agents_num)]
for a_id in range(agents_num):
    agp = mppi_ca_cpp.convert_agent_params(agents_params[a_id])
    alp = mppi_ca_cpp.convert_alg_params(alg_params)
    controllers[a_id].set_agent_params(agp)
    controllers[a_id].set_alg_params(alp)

agents_states = np.array([[0, 0, 0, 0, 0],
                         [1, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0],
                         [-1, 0, 0, 0, 0],
                         [0, -1, 0, 0, 0]], dtype=np.float64)

goal_states =  np.array([[10, 10, 0],
                         [-10, 0, 0],
                         [0, 10, 0],
                         [-10, -10, 0],
                         [0, -10, 0]], dtype=np.float64)

print("Init done")
common_time = 0

for a_id in range(agents_num):
    neighbours_id = set(range(agents_num))
    neighbours_id.remove(a_id)
    neighbours_id = list(neighbours_id)
    neighbours_states = agents_states[neighbours_id]
    neighbours_states = neighbours_states[:, np.array([True, True, False, True, True])]
    start = timeit.default_timer()
    u, traj = controllers[a_id].next_step(agents_states[a_id], neighbours_states, goal_states[a_id])
    stop = timeit.default_timer()
    time = (stop - start) * 1000
    print(f"Agent {a_id} step done. New control: {u}, Time: {time} ms")
    common_time += time

print(f"Av.Time: {common_time / agents_num} ms")

