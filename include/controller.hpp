#ifndef MPPI_CA_CPP_CONTROLLER_HPP
#define MPPI_CA_CPP_CONTROLLER_HPP

#include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xfixed.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <iostream>
#include <ctime>

#include "consts.hpp"
#include "diff_drive.hpp"
#include "geometry.hpp"

namespace mppica
{
	using std::cout;
	using std::endl;


	class Controller
	{
		public:
			Controller();

			auto nextStep(const xt::xtensor<double, 1> &curr_state, const xt::xtensor<double, 2> &neighbours_states,
						  const xt::xtensor<double, 1> &goal_state) -> xt::xtensor<double, 1>;

			void reset();

		private:
			void resetControlSeq();

			void resetTrajectories(const xt::xtensor<double, 1> &initial_state);

			void resetSamples();

			void generateSamples();

			void limitControl(size_t batch, size_t time_step);

			void updateControl(size_t batch, size_t time_step);

			void updateTrajectory(size_t batch, size_t time_step);

			void updateControlSequence();

			auto getCurrentControl() -> xt::xtensor<double, 1>;

			void computeRunningCosts(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
									 const xt::xtensor<double, 1> &goal_state);

			void commonStateCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
								 const xt::xtensor<double, 1> &goal_state);

			void computeTerminalCosts(size_t batch, const xt::xtensor<double, 1> &goal_state);

			void commonTerminalCost(size_t batch, const xt::xtensor<double, 1> &goal_state);

			void shiftControlSequence();

			bool checkCollisionFree();

			bool generateStopControl();

			void updateStopComputationConditions();

			bool computationShouldStop();

			void computeMainTrajectory();


			bool ma_cost_en;

			size_t batch_size;
			size_t time_steps;
			xt::xtensor<double, 1> mean;
			xt::xtensor<double, 2> cov;
			xt::xtensor<double, 2> control_limits;
			xt::xtensor<double, 1> initial_values;

			double dt;
			double lambda;

			bool safe_control_not_found;

			xt::xtensor<double, 3> sampled_controls;
			xt::xtensor<double, 3> samples;
			xt::xtensor<double, 2> control_sequence;
			xt::xtensor<double, 3> generated_trajectories;
			xt::xtensor<double, 1> traj_costs;

			double common_run_cost_weight;
			double common_terminal_cost_weight;
			xt::xtensor<double, 2> terminal_weight;

			DiffDrive dyn_model; //TODO Generalize for other models
	};

}


#endif //MPPI_CA_CPP_CONTROLLER_HPP
