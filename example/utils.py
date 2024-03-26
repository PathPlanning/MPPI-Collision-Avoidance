import numpy as np
import os
import time
import sys
from scipy.spatial import KDTree

from params_names import *
from os.path import dirname

sys.path.append(dirname("../build/release/mppi_ca_cpp.so"))
import mppi_ca_cpp



def circle_instance(circ_r, agents_num):
    start_states = np.zeros((agents_num, 5), dtype=np.float64)
    goal_states = np.zeros((agents_num, 5), dtype=np.float64)
    ang_steps = np.linspace(0.0, 2*np.pi, agents_num+1)
    
    for i, ang in enumerate(ang_steps):
        if i == agents_num:
            break

        st_x = circ_r * np.cos(ang)
        st_y = circ_r * np.sin(ang)

        g_x = circ_r * np.cos(ang + np.pi)
        g_y = circ_r * np.sin(ang + np.pi)

        dx = g_x - st_x
        dy = g_y - st_y

        st_theta = np.arctan2(dy, dx)
        g_theta = st_theta

        start_states[i] = np.array([st_x, st_y, st_theta, 0.0, 0.0])
        goal_states[i] = np.array([g_x, g_y, g_theta, 0.0, 0.0])
        
    return start_states, goal_states


def get_neighbors(a_pos_id, all_pos, r):
    tree = KDTree(all_pos)
    neighbors_ids = tree.query_ball_point(all_pos[a_pos_id], r)
    if a_pos_id in neighbors_ids:
        neighbors_ids.remove(a_pos_id)
    return neighbors_ids


def check_collisions(agents_states, agent_sizes):
    collisions = 0
    for a1 in range(len(agents_states)):
        for a2 in range(len(agents_states)):
            if a1 == a2:
                continue
            if np.linalg.norm(agents_states[a1, 0:2] - agents_states[a2, 0:2]) < (agent_sizes[a1] + agent_sizes[a2]):
                collisions += 1
    return collisions / 2 

def launch_mppi(
    start_states,
    goal_states,
    agents_num,
    agents_params,
    alg_params,
    exp_params,
    dyn_models,
    save_logs=True,
    save_traj_lines=False,
    seed=None,
    positions_std=0.0,
    direction_std=0.0,
    velocities_std=0.0,
):

    if seed is None:
        seed = int.from_bytes(os.urandom(4), byteorder="little")
    np.random.seed(seed)

    # if type(exp_params) is not dict:
    #     exp_params = exp_params.__dict__
    max_sim_steps = exp_params[MAX_EXP_STEPS]
    dt = exp_params[EXP_TIME_STEP]
    goal_xy_offset = exp_params[XY_GOAL_TOLERANCE]

    agents_sizes = []
    agents_r_vis = []
    controllers = [mppi_ca_cpp.Controller() for a_id in range(agents_num)]

    for a_id in range(agents_num):
        # if type(agents_params[a_id]) is not dict:
        #     agents_params[a_id] = agents_params[a_id].__dict__

        agp = mppi_ca_cpp.convert_agent_params(agents_params[a_id])
        alp = mppi_ca_cpp.convert_alg_params(alg_params)
        controllers[a_id].set_agent_params(agp)
        controllers[a_id].set_alg_params(alp)
        agents_sizes.append(agents_params[a_id][AGENT_SIZE])
        agents_r_vis.append(agents_params[a_id][VISIBILITY_RADIUS])

    agents_states = start_states.copy()
    new_states = agents_states.copy()
    agents_sizes = np.array(agents_sizes)

    log_state = [start_states]
    log_traj = []
    collisions = 0
    steps = 0

    t_start = time.time()
    for iter in range(max_sim_steps):
        step_traj = []
        steps += 1

        for a_id in range(agents_num):

            curr_state = new_states[a_id, 0:5].copy()
            goal_state = goal_states[a_id, 0:3].copy()
            neighbours_id = get_neighbors(
                a_id, agents_states[:, 0:2], agents_r_vis[a_id]
            )
            neighbours_states = agents_states[neighbours_id]
            neighbours_states = neighbours_states[
                :, np.array([True, True, False, True, True])
            ]
            neighbours_sizes = agents_sizes[neighbours_id]

            noized_curr_state = curr_state.copy()
            noized_neighbours_states = neighbours_states.copy()


            if positions_std != 0.0:
                noized_curr_state[0:2] = np.random.normal(
                    curr_state[0:2], positions_std, 2
                )
                for i in range(len(neighbours_states)):
                    noized_neighbours_states[i, 0:2] = np.random.normal(
                        neighbours_states[i, 0:2], positions_std, 2
                    )

            if direction_std != 0.0:
                noized_curr_state[2] = np.random.normal(curr_state[2], direction_std, 1)

            if velocities_std != 0.0:
                noized_curr_state[3:] = np.random.normal(
                    curr_state[3:], velocities_std, 2
                )
                for i in range(len(neighbours_states)):
                    noized_neighbours_states[i, 2:] = np.random.normal(
                        neighbours_states[i, 2:], velocities_std, 2
                    )

            step_res, traj = controllers[a_id].next_step(
                noized_curr_state,
                noized_neighbours_states,
                neighbours_sizes,
                goal_state,
            )
            u = step_res
            if save_logs and save_traj_lines:
                step_traj.append(traj)

            new_states[a_id, 0:3] = dyn_models[a_id].state_shift(
                curr_state[0:3], dt
            ) + dyn_models[a_id].dynamic_model(curr_state[0:3], u, dt)

            new_states[a_id, 3:5] = (new_states[a_id, 0:2] - curr_state[0:2]) / dt

        agents_states = new_states.copy()

        if save_logs:
            log_state.append(agents_states.copy())
            if save_traj_lines:
                log_traj.append(step_traj.copy())

        collisions += check_collisions(agents_states, agents_sizes)

        done = True
        for a_id in range(agents_num):
            if (
                np.linalg.norm(agents_states[a_id, 0:2] - goal_states[a_id, 0:2])
                > goal_xy_offset
            ):
                done = False
                break

        if done:
            t_end = time.time()
            t_dur = t_end - t_start

            if save_logs:
                if save_traj_lines:
                    return (
                        True,
                        collisions,
                        steps,
                        t_dur,
                        log_state,
                        log_traj
                    )
                else:
                    return True, collisions, steps, t_dur, log_state
            else:
                return True, collisions, steps, t_dur

    t_end = time.time()
    t_dur = t_end - t_start

    if save_logs:
        if save_traj_lines:
            return False, collisions, steps, t_dur, log_state, log_traj
        else:
            return False, collisions, steps, t_dur, log_state
    else:
        return False, collisions, steps, t_dur
