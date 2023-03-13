#include "controller.hpp"
#include <chrono>
#include <vector>

namespace mppica {

	Controller::Controller() {
		xt::random::seed(time(NULL));
		setAlgParams(MPPIParams());
		setAgentParams(AgentParams());
		dyn_model = DiffDrive();
		reset();
	}

	auto Controller::nextStep(const xt::xtensor<double, 1> &curr_state, const xt::xtensor<double, 2> &neighbors_states,
							  const xt::xtensor<double, 1> &goal_state) -> xt::xtensor<double, 1> {

		do {
			resetTrajectories(curr_state);

			if(ma_cost_en) {
				propagateNeighboursTrajectories(neighbors_states);
			}

			generateSamples();

			for (size_t batch = 0; batch < batch_size; ++batch) {
				for (size_t t_step = 0; t_step < time_steps - 1; ++t_step) {
					updateControl(batch, t_step);
					limitControl(batch, t_step);
					updateTrajectory(batch, t_step);
					computeRunningCosts(batch, t_step+1, curr_state, goal_state);
				}
				computeTerminalCosts(batch, goal_state);
			}
			updateControlSequence();
			safe_control_not_found = checkCollisionFree();
		} while (safe_control_not_found and not computationShouldStop());


		if (safe_control_not_found) {
			generateStopControl();
			std::cerr << "No safe trajectory";
		}


		auto result = getCurrentControl();
		shiftControlSequence();
		return result;
	}

	void Controller::reset() {
		resetTrajectories(xt::zeros<double>({DD_STATE_DIM}));
		resetControlSeq();
		resetSamples();
		resetNeighborsTrajectories();
	}

	void Controller::resetTrajectories(const xt::xtensor<double, 1> &initial_state) {
		generated_trajectories = xt::zeros<double>({batch_size, time_steps, DD_STATE_DIM});
		traj_costs = xt::zeros<double>({batch_size});
		xt::view(generated_trajectories, xt::all(), 0) = initial_state;
	}

	void Controller::resetControlSeq() {
		control_sequence = xt::empty<double>({time_steps - 1, DD_CONTROL_DIM});
		xt::view(control_sequence, xt::all(), xt::all()) = initial_values;
	}

	void Controller::resetSamples() {
		samples = xt::zeros<double>({batch_size, time_steps - 1, DD_CONTROL_DIM});
		sampled_controls = xt::zeros<double>({batch_size, time_steps - 1, DD_CONTROL_DIM});
	}

	void Controller::generateSamples() {
		// TODO multivariate normal sampling for general cases
		xt::xtensor<double, 2>::shape_type control_row_shape = {batch_size, time_steps - 1};
		for (size_t i = 0; i < DD_CONTROL_DIM; ++i) {
			xt::view(samples, xt::all(), xt::all(), i) = xt::random::randn<double>(control_row_shape, mean[i],
																				   sample_std[{i, i}]);
		}
	}

	void Controller::updateControl(size_t batch, size_t time_step) {
		xt::view(sampled_controls, batch, time_step, xt::all()) = xt::view(control_sequence, time_step, xt::all()) +
																  xt::view(samples, batch, time_step, xt::all());
	}

	void Controller::limitControl(size_t batch, size_t time_step) {
		for (size_t i = 0; i < DD_CONTROL_DIM; ++i) {
			sampled_controls[{batch, time_step, i}] = xt::clip(xt::view(sampled_controls, batch, time_step, i),
															   control_limits[{i, static_cast<size_t>(0)}],
															   control_limits[{i, static_cast<size_t>(1)}]);
		}

	}

	void Controller::updateTrajectory(size_t batch, size_t time_step) {
		xt::view(generated_trajectories, batch, time_step + 1) = xt::view(generated_trajectories, batch, time_step) +
																 dyn_model.shift(xt::view(generated_trajectories, batch,
																						  time_step), dt) +
																 dyn_model.dynamic(
																		 xt::view(generated_trajectories, batch,
																				  time_step),
																		 xt::view(sampled_controls, batch, time_step),
																		 dt);
	}

	void Controller::computeRunningCosts(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
										 const xt::xtensor<double, 1> &goal_state) {
		commonStateCost(batch, time_step, initial_state, goal_state);
		if(ma_cost_en) multiAgentCost(batch, time_step, goal_state);
	}

	void Controller::commonStateCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
									 const xt::xtensor<double, 1> &goal_state) {
		auto dist = point_to_line_segment_distance(xt::view(initial_state, xt::range(0, 2)),
												   xt::view(goal_state, xt::range(0, 2)),
												   xt::view(generated_trajectories, batch, time_step, xt::range(0, 2)));
		traj_costs[batch] += common_run_cost_weight * dist;
	}

	void Controller::computeTerminalCosts(size_t batch, const xt::xtensor<double, 1> &goal_state) {
		commonTerminalCost(batch, goal_state);
	}

	void Controller::commonTerminalCost(size_t batch, const xt::xtensor<double, 1> &goal_state) {
		auto last_pos = xt::view(generated_trajectories, batch, -1, xt::range(0, 2));
		auto goal_pos = xt::view(goal_state, xt::range(0, 2));
		auto dist = xt::linalg::norm(last_pos - goal_pos, 2);

		traj_costs[batch] += common_terminal_cost_weight * dist;
	}

	bool Controller::generateStopControl() {
		// TODO
		return false;
	}

	bool Controller::computationShouldStop() {
		// TODO
		return false;
	}

	void Controller::updateStopComputationConditions() {
		// TODO
	}

	void Controller::updateControlSequence() {
		// TODO
		auto beta = xt::amin(traj_costs, xt::evaluation_strategy::immediate);
		auto eta =
				xt::sum(xt::exp(-1. / lambda * (traj_costs - beta)), {0}, xt::evaluation_strategy::immediate) + 1e-10;
		auto weights = xt::eval(xt::exp(-1. / lambda * (traj_costs - beta)) / eta);

		control_sequence = xt::sum(xt::view(weights, xt::all(), xt::newaxis(), xt::newaxis()) * sampled_controls, {0},
								   xt::evaluation_strategy::immediate);
	}

	auto Controller::getCurrentControl() -> xt::xtensor<double, 1> {
		return xt::view(control_sequence, 0);
	}

	void Controller::shiftControlSequence() {
		using namespace xt::placeholders;
		xt::view(control_sequence, xt::range(_, -1), xt::all()) = xt::view(control_sequence, xt::range(1, _),
																		   xt::all());
		xt::view(control_sequence, -1, xt::all()) = initial_values;
	}

	void Controller::computeMainTrajectory() {
		// TODO
	}

	bool Controller::checkCollisionFree() {
		// TODO
		return false;
	}

	void Controller::setAlgParams(MPPIParams alg_params) {
		time_steps = alg_params.time_steps;
		batch_size = alg_params.batch_size;
		mean = alg_params.mean;
		sample_std = xt::sqrt(alg_params.cov);
		initial_values = alg_params.initial_values;
		dt = alg_params.dt;
		lambda = alg_params.lambda;
		common_run_cost_weight = alg_params.common_run_cost_weight;
		common_terminal_cost_weight = alg_params.common_terminal_cost_weight;
		ma_cost_en = alg_params.ma_cost_en;
		max_neighbors_num = alg_params.max_neighbors_num;
		ma_dist_weight = alg_params.ma_dist_weight;
		ma_collision_value = alg_params.ma_collision_value;
		ma_dist_threshold = alg_params.ma_dist_threshold;
		reset();
	}

	void Controller::setAgentParams(AgentParams ag_params) {
		agent_size = ag_params.size;
		vis_radius = ag_params.vis_radius;
		control_limits = {{ag_params.dd_v_min, ag_params.dd_v_max},
						  {ag_params.dd_w_min, ag_params.dd_w_max}};
		reset();
	}

	void Controller::propagateNeighboursTrajectories(const xt::xtensor<double, 2> &neighbors_states) {

		if (neighbors_states.shape(0) == 0) {
			neighbors_num = 0;
			return;
		}

		neighbors_num = (max_neighbors_num > neighbors_states.shape(0)) ? neighbors_states.shape(0) : max_neighbors_num;
		xt::view(neighbors_traj, xt::range(0, neighbors_num), 0, xt::all()) =
				xt::view(neighbors_states, xt::range(0, neighbors_num), xt::range(0, POSITION_DIM));

		for (size_t t = 1; t < time_steps; ++t)
			xt::view(neighbors_traj, xt::range(0, neighbors_num), t, xt::all()) = xt::view(neighbors_traj, xt::range(0, neighbors_num), 0, xt::all()) +
																xt::view(neighbors_states, xt::range(0, neighbors_num), xt::range(POSITION_DIM, _)) * t * dt;
	}

	void Controller::resetNeighborsTrajectories() {
		neighbors_traj = xt::zeros<double>({max_neighbors_num, time_steps, POSITION_DIM});
	}

	void Controller::multiAgentCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &goal_state) {
		if (neighbors_num == 0)
			return;

		auto curr_pos = xt::view(generated_trajectories, batch, 0, xt::range(0, POSITION_DIM));
		auto goal_pos = xt::view(goal_state, xt::range(0, 2));
		auto goal_dist = xt::linalg::norm(curr_pos - goal_pos, 2);
		if (goal_dist < 0.5 and time_step > 3) // TODO do params
			return;

		double min_dist = std::numeric_limits<double>::max();

		for (int agent = 0; agent < neighbors_num; ++agent) {
			double dist = xt::linalg::norm(
					xt::view(neighbors_traj, agent, time_step, xt::all()) -
					xt::view(generated_trajectories, batch, time_step, xt::range(0, POSITION_DIM))
					, 2);
			if (dist < min_dist)
				min_dist = dist;

			if (dist < (2 * agent_size)) {
				traj_costs[batch] = ma_collision_value;
				return;
			}
		}
		if (min_dist > ma_dist_threshold)
			return;
		traj_costs[batch] += ma_dist_weight * (2 * agent_size / min_dist);
	}

}