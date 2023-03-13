# Agents parameters
DIFF_DRIVE_V_MAX    = 'v_max'
DIFF_DRIVE_V_MIN    = 'v_min'
DIFF_DRIVE_W_MAX    = 'w_max'
DIFF_DRIVE_W_MIN    = 'w_min'
AGENT_SIZE          = 'size'
VISIBILITY_RADIUS   = 'r_vis'

# MPPI parameters
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



# Experiment parameters
EXP_TIME_STEP       = 'dt'
XY_GOAL_TOLERANCE   = 'xy_goal_tolerance'
MAX_EXP_STEPS       = 'max_steps'

import sys
from os.path import dirname
sys.path.append(dirname("/home/sadergachev/Documents/projects/multi-agent-nav/src/mppi-ca-cpp/cmake-build-release/mppi_ca_cpp.cpython-310-x86_64-linux-gnu.so"))

import mppi_ca_cpp
import numpy as np
import timeit

print("Start")

max_sim_steps   = 500
dt              = 0.1
goal_xy_offset  = 0.3

default_r_vis   = 0.0
default_ag_size = 0.3
default_v_max   = 1.0
default_v_min   = -1.0
default_w_max   = 2.0
default_w_min   = -2.0

alg_params = dict()
default_agent_params = dict()
exp_params = dict()


p = mppi_ca_cpp.Controller()
default_agent_params[VISIBILITY_RADIUS] = default_r_vis
default_agent_params[AGENT_SIZE] = default_ag_size
default_agent_params[DIFF_DRIVE_V_MAX] = default_v_max
default_agent_params[DIFF_DRIVE_V_MIN] = default_v_min
default_agent_params[DIFF_DRIVE_W_MAX] = default_w_max
default_agent_params[DIFF_DRIVE_W_MIN] = default_w_min


alg_params[SAMPLE_BATCH_SIZE]   = 500
alg_params[SAMPLE_TIME_STEPS]   = 10
alg_params[TIME_STEP]           = dt
alg_params[INIT_VALUE]          = np.array([0.0, 0.0])
alg_params[CONTROL_MEAN]        = np.array([100.0, 0.0])
alg_params[CONTROL_COV]         = np.array([0.01, 0.0, 0.0, 0.05], dtype=np.float64).reshape((2, 2))
alg_params[RUNNING_COST_WEIGH]  = 10.0 / alg_params[SAMPLE_TIME_STEPS] # np.array([100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64).reshape((3, 3))
alg_params[TERMINAL_WEIGHT]     = 500
alg_params[SOFTMAX_TEMPERATURE] = 0.1
alg_params[MAX_SAFE_DIST_STEP]  = 0
alg_params[MAX_BVC_COST_STEP]   = 0
alg_params[MA_COST_EN]          = False
alg_params[IMPORTANCE_SAMPL_EN] = False
alg_params[BVC_WEIGHT]          = 0
alg_params[MA_DIST_WEIGHT]      = 0
alg_params[MA_COLLISION_WEIGHT] = 0
alg_params[BVC_W_BASE]          = 0.0
alg_params[BVC_W_MEAN]          = 0.0
alg_params[BVC_W_STD]           = 0.0
alg_params[MA_DIST_THREASHOLD]  = 0.0

agp = mppi_ca_cpp.convert_agent_params(default_agent_params)
alp = mppi_ca_cpp.convert_alg_params(alg_params)
p.set_agent_params(agp)
p.set_alg_params(alp)

print("Init done")
start = timeit.default_timer()
a = p.next_step(np.ones(3), np.ones((10, 4)), np.ones(3))
stop = timeit.default_timer()
print('Time: ', stop - start)
print("Step done", a)