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


	params.batch_size = params_dict["batch_size"].cast<size_t>();;
	params.time_steps = params_dict["time_steps"].cast<size_t>();;
	params.dt = params_dict["dt"].cast<double>();
	params.initial_values = params_dict["init_value"].cast<xt::xtensor<double, 1>>();
	params.mean = params_dict["control_mean"].cast<xt::xtensor<double, 1>>();
	params.cov = params_dict["control_cov"].cast<xt::xtensor<double, 2>>();

	params.common_run_cost_weight = params_dict["dist_to_goal_weight"].cast<double>();
	params.common_terminal_cost_weight = params_dict["terminal_weight"].cast<double>();
	params.lambda = params_dict["mppi_lambda"].cast<double>();
	params.ma_cost_en = params_dict["ma_cost_en"].cast<bool>();
	params.max_neighbors_num = params_dict["max_neighbors_num"].cast<size_t>();
	params.ma_dist_weight = params_dict["ma_dist_weight"].cast<double>();
	params.ma_collision_value = params_dict["ma_collision_weight"].cast<double>();
	params.ma_dist_threshold = params_dict["ma_dist_threashhold"].cast<double>();

	return params;
}
