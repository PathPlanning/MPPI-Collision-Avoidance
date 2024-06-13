#include "converters.hpp"


mppica::AgentParams convertAgentParams(const py::dict& params_dict)
{
mppica::AgentParams params;

params.size = params_dict["size"].cast<double>();
params.vis_radius = params_dict["r_vis"].cast<double>();

params.dd_v_max = params_dict["v_max"].cast<double>();
params.dd_v_min = params_dict["v_min"].cast<double>();
params.dd_w_max = params_dict["w_max"].cast<double>();
params.dd_w_min = params_dict["w_min"].cast<double>();

return params;
}

mppica::MPPIParams convertAlgParams(const py::dict &params_dict) {
	mppica::MPPIParams params;

	params.batch_size = getValue(params_dict, "batch_size", params.batch_size);
	params.time_steps = getValue(params_dict, "time_steps", params.time_steps);
	params.dt = getValue(params_dict, "dt", params.dt);
	params.initial_values = getValue(params_dict, "init_value", params.initial_values);
	params.mean = getValue(params_dict, "control_mean", params.mean);
	params.cov = getValue(params_dict, "control_cov", params.cov);
	params.run_cost_weight = getValue(params_dict, "dist_to_goal_weight", params.run_cost_weight);
	params.terminal_cost_weight = getValue(params_dict, "terminal_weight", params.terminal_cost_weight);
	params.lambda = getValue(params_dict, "mppi_lambda", params.lambda);
	params.ma_cost_en = getValue(params_dict, "ma_cost_en", params.ma_cost_en);
	params.ma_dist_weight = getValue(params_dict, "ma_dist_weight", params.ma_dist_weight);
	params.ma_collision_value = getValue(params_dict, "ma_collision_weight", params.ma_collision_value);
	params.ma_dist_threshold = getValue(params_dict, "ma_dist_threashhold", params.ma_dist_threshold);
	params.k_alpha = getValue(params_dict, "k_alpha", params.k_alpha);
	params.rad_eps = getValue(params_dict, "rad_eps", params.rad_eps);
	params.velocity_cost_weight = getValue(params_dict, "velocity_cost_weight", params.velocity_cost_weight);
	params.terminal_lookahead_distance = getValue(params_dict, "terminal_lookahead_distance", params.terminal_lookahead_distance);
	params.ma_en_close_to_goal_dist = getValue(params_dict, "ma_en_close_to_goal_dist", params.ma_en_close_to_goal_dist);
	params.ma_en_close_to_goal_ts = getValue(params_dict, "ma_en_close_to_goal_ts", params.ma_en_close_to_goal_ts);
	params.orca_time_boundary = getValue(params_dict, "orca_time_boundary", params.orca_time_boundary);
	params.orca_responsibility_factor = getValue(params_dict, "orca_responsibility_factor", params.orca_responsibility_factor);
	params.ma_exclude_collision_traj = getValue(params_dict, "ma_exclude_collision_traj", params.ma_exclude_collision_traj);
	params.velocity_cost_close_to_goal_dist = getValue(params_dict, "velocity_cost_close_to_goal_dist", params.velocity_cost_close_to_goal_dist);
	params.car_like_dyn = getValue(params_dict, "car_like_dyn", params.car_like_dyn);
	if (params.car_like_dyn) {
		params.dist_between_axles = getValue(params_dict, "dist_between_axles", params.dist_between_axles);
	}
	return params;
}

template <typename T>
T getValue(const py::dict& data, const std::string &key, const T &default_value) {
	if (data.contains(key)) {
		return data[key.c_str()].cast<T>();
	}
	std::cerr << "The value of \"" << key << "\" parameter not found\n";
	return default_value;
}
