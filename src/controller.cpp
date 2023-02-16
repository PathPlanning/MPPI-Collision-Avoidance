#include "controller.hpp"
#include <chrono>
#include <vector>

namespace mppica
{
	Controller::Controller()
	{
		xt::random::seed(time(NULL));
		ma_cost_en = true;
		time_steps = 20;
		batch_size = 200;
		mean = xt::zeros<double>({DD_CONTROL_DIM});
		cov = xt::zeros<double>({DD_CONTROL_DIM, DD_CONTROL_DIM});
		initial_values = xt::zeros<double>({DD_CONTROL_DIM});
		dt = 0.1;
		lambda = 0.1;

		cov[{0, 0}] = 0.01;
		cov[{1, 1}] = 0.05;
		control_limits = {{-1.0, 1.0},
						  {-2.0, 2.0}};
		dyn_model = DiffDrive();
		common_run_cost_weight = 10.0 / time_steps;
		common_terminal_cost_weight = 10.0;

		reset();
	}

	auto Controller::nextStep(const xt::xtensor<double, 1> &curr_state, const xt::xtensor<double, 2> &neighbours_states,
							  const xt::xtensor<double, 1> &goal_state) -> xt::xtensor<double, 1>
	{
		auto start = std::chrono::steady_clock::now();
		do {
			resetTrajectories(curr_state);
			generateSamples();

			for (size_t batch = 0; batch < batch_size; ++batch) {
				for (size_t t_step = 0; t_step < time_steps - 1; ++t_step) {
					updateControl(batch, t_step);
					limitControl(batch, t_step);
					updateTrajectory(batch, t_step);
					computeRunningCosts(batch, t_step, curr_state, goal_state);
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
		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << "elapsed time inside: " << elapsed_seconds.count() << " s\n";
		std::cout << control_sequence << endl;
		return result;
	}

	void Controller::reset()
	{
		resetTrajectories(xt::zeros<double>({DD_STATE_DIM}));
		resetControlSeq();
		resetSamples();
	}

	void Controller::resetTrajectories(const xt::xtensor<double, 1> &initial_state)
	{
		generated_trajectories = xt::zeros<double>({batch_size, time_steps, DD_STATE_DIM});
		traj_costs = xt::zeros<double>({batch_size});
		xt::view(generated_trajectories, xt::all(), 0) = initial_state;
	}


	void Controller::resetControlSeq()
	{
		// TODO fill with initial value
//		control_sequence = xt::zeros<double>({time_steps - 1, DD_CONTROL_DIM});
		control_sequence = xt::empty<double>({time_steps - 1, DD_CONTROL_DIM});
		xt::view(control_sequence, xt::all(), xt::all()) = initial_values;
	}


	void Controller::resetSamples()
	{
		samples = xt::zeros<double>({batch_size, time_steps - 1, DD_CONTROL_DIM});
		sampled_controls = xt::zeros<double>({batch_size, time_steps - 1, DD_CONTROL_DIM});
	}

	void Controller::generateSamples()
	{
		// TODO multivariate normal sampling for general cases
		xt::xtensor<double, 2>::shape_type control_row_shape = {batch_size, time_steps - 1};
		for (size_t i = 0; i < DD_CONTROL_DIM; ++i)
		{
			xt::view(samples, xt::all(), xt::all(), i) = xt::random::randn<double>(control_row_shape, mean[i],
																				   cov[{i, i}]);
		}
	}

	void Controller::updateControl(size_t batch, size_t time_step)
	{
		xt::view(sampled_controls, batch, time_step, xt::all()) = xt::view(control_sequence, time_step, xt::all()) +
																  xt::view(samples, batch, time_step, xt::all());
	}

	void Controller::limitControl(size_t batch, size_t time_step)
	{
		for (size_t i = 0; i < DD_CONTROL_DIM; ++i)
		{
			sampled_controls[{batch, time_step, i}] = xt::clip(xt::view(sampled_controls, batch, time_step, i),
															   control_limits[{i, static_cast<size_t>(0)}],
															   control_limits[{i, static_cast<size_t>(1)}]);
		}

	}


	void Controller::updateTrajectory(size_t batch, size_t time_step)
	{
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
										 const xt::xtensor<double, 1> &goal_state)
	{
		commonStateCost(batch, time_step, initial_state, goal_state);
	}

	void Controller::commonStateCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
									 const xt::xtensor<double, 1> &goal_state)
	{
		auto dist = point_to_line_segment_distance(xt::view(initial_state, xt::range(0, 2)),
												   xt::view(goal_state, xt::range(0, 2)),
												   xt::view(generated_trajectories, batch, time_step, xt::range(0, 2)));
		traj_costs[batch] += common_run_cost_weight * dist;
	}

	void Controller::computeTerminalCosts(size_t batch, const xt::xtensor<double, 1> &goal_state)
	{
		commonTerminalCost(batch, goal_state);
	}


	void Controller::commonTerminalCost(size_t batch, const xt::xtensor<double, 1> &goal_state)
	{
		auto last_pos = xt::view(generated_trajectories, batch, -1, xt::range(0, 2));
		auto goal_pos = xt::view(goal_state, xt::range(0, 2));
		auto dist = xt::linalg::norm(last_pos - goal_pos, 2);
		traj_costs[batch] += common_terminal_cost_weight * dist;
	}

	bool Controller::generateStopControl()
	{
		// TODO
		return false;
	}

	bool Controller::computationShouldStop()
	{
		// TODO
		return false;
	}

	void Controller::updateStopComputationConditions()
	{
		// TODO
	}


	void Controller::updateControlSequence()
	{
		// TODO
		auto beta = xt::amin(traj_costs, xt::evaluation_strategy::immediate);
		auto eta =
				xt::sum(xt::exp(-1. / lambda * (traj_costs - beta)), {0}, xt::evaluation_strategy::immediate) + 1e-10;
		auto weights = xt::eval(xt::exp(-1. / lambda * (traj_costs - beta)) / eta);

		control_sequence = xt::sum(xt::view(weights, xt::all(), xt::newaxis(), xt::newaxis()) * sampled_controls, {0},
								   xt::evaluation_strategy::immediate);
	}

	auto Controller::getCurrentControl() -> xt::xtensor<double, 1>
	{
		return xt::view(control_sequence, 0);
	}


	void Controller::shiftControlSequence()
	{
		// TODO fill with initial value
		using namespace  xt::placeholders;
		xt::view(control_sequence, xt::range(_, -1), xt::all()) = xt::view(control_sequence, xt::range(1, _), xt::all());
		xt::view(control_sequence, -1, xt::all()) = initial_values;
	}


	void Controller::computeMainTrajectory()
	{
		// TODO
	}

	bool Controller::checkCollisionFree()
	{
		// TODO
		return false;
	}

}