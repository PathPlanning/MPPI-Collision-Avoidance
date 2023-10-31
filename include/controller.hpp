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
#include <limits>
#include <cmath>
#include <memory>
#include <random>

#include "ortools/linear_solver/linear_solver.h"

#include "consts.hpp"
#include "dynamics.hpp"
#include "geometry.hpp"
#include "orca_lines.hpp"

namespace mppica {
	using std::cout;
	using std::endl;
	using namespace xt::placeholders;


	struct MPPIParams {
		size_t batch_size = 100;
		size_t time_steps = 10;
		double dt = 0.1;
		xt::xtensor<double, 1> initial_values = xt::zeros<double>({2});
		xt::xtensor<double, 1> mean = xt::zeros<double>({2});;
		xt::xtensor<double, 2> cov = xt::eye<double>(2);

		double common_run_cost_weight = 1.0;
		double common_terminal_cost_weight = 1.0;
		double lambda = 0.1;
		bool ma_cost_en = true;
		size_t max_neighbors_num = 10;
		double ma_dist_weight = 1000.0;
		double ma_collision_value = 10000.0;
		double ma_dist_threshold = 2.0;
		int max_safe_distr_step = 0;
		bool car_like_dyn = false;
        double alpha; // TODO: Sampling with initial mean value in alpha% of batches
	};

	struct AgentParams {

		double size = 1.0;
		double vis_radius = 1.0;
		double dd_v_max = 1.0;
		double dd_v_min = -1.0;
		double dd_w_max = 2.0;
		double dd_w_min = -2.0;
	};


	class Controller {
		public:
			Controller();

			auto nextStep(const xt::xtensor<double, 1> &curr_state, const xt::xtensor<double, 2> &neighbors_states,
						  const xt::xtensor<double, 1> &goal_state) -> std::tuple<xt::xtensor<double, 1>, xt::xtensor<double, 3>>;

			void setAgentParams(AgentParams ag_params);

			void setAlgParams(MPPIParams alg_params);

			void reset();

		private:
			void resetControlSeq();

			void resetTrajectories(const xt::xtensor<double, 1> &initial_state);

			void resetSamples();

			void resetNeighborsTrajectories();

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

			void propagateNeighboursTrajectories(const xt::xtensor<double, 2> &neighbors_states);


			void multiAgentCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &goal_state);

			void computeLinearConstraintsForState(const xt::xtensor<double, 1> &state,
												  const xt::xtensor<double, 2> &neighbors_states);

			void resampleInitialWithConstraints();


			std::tuple<bool, double, double> computeNewDistr(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control);

			std::tuple<xt::xtensor<double, 1>, xt::xtensor<double, 1>> translateConstraintsToControlBounds(const xt::xtensor<double, 1> &state);

			bool computationShouldStop(); // TODO
			void updateStopComputationConditions(); // TODO
			void computeMainTrajectory(); // TODO


			size_t step; // For debug purpuses
			size_t controller_id; // For debug purpuses

			bool ma_cost_en;
			size_t max_neighbors_num;
			size_t neighbors_num;
			double ma_dist_weight;
			double ma_collision_value;
			double ma_dist_threshold;
			int max_safe_distr_step;

			size_t batch_size;
			size_t time_steps;
			xt::xtensor<double, 1> mean;
			xt::xtensor<double, 2> sample_std;
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
			xt::xtensor<double, 3> neighbors_traj;
			xt::xtensor<double, 2> linear_constraints;
			std::vector<ORCALine> orca_lines;
			bool orca_complete = true;



			double common_run_cost_weight;
			double common_terminal_cost_weight;

			double agent_size;
			double vis_radius;
			double rad_eps;

			bool use_car_like_dyn;

			DiffDrive dyn_model; //TODO Generalize for other models
			CarLike car_like_dyn;


	};

}


#endif //MPPI_CA_CPP_CONTROLLER_HPP