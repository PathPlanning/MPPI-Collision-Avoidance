#ifndef MPPI_CA_CPP_ORCA_LINES_HPP
#define MPPI_CA_CPP_ORCA_LINES_HPP

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

#include "consts.hpp"
#include "geometry.hpp"


struct ORCALine {
	xt::xtensor<double, 1> dir = xt::zeros<double>({2});
	xt::xtensor<double, 1> point_lies_on = xt::zeros<double>({2});;
};

struct ORCAParams {
	size_t agents_max_num;
	double agent_size;
	double time_boundary;
	double time_step;
	double responsibility_factor = 0.5;
	bool opt_zero_vel = true;
};

std::vector<ORCALine> computeORCALines(ORCAParams params, xt::xtensor<double, 1> agent_state, xt::xtensor<double, 2> neighbors_states,
					  xt::xtensor<double, 1> neighbors_sizes);

xt::xtensor<double, 2> computeORCALinesXT(size_t agents_max_num,
										  double agent_size,
										  double time_boundary,
										  double time_step,
										  double responsibility_factor,
										  bool opt_zero_vel,
										  xt::xtensor<double, 1> agent_state,
										  xt::xtensor<double, 2> neighbors_states,
										  xt::xtensor<double, 1> neighbors_sizes);


bool linearProgram1(const std::vector<ORCALine> &lines, unsigned long curr, float radius, const xt::xtensor<double, 1> &opt_velocity,
					bool direction_opt, xt::xtensor<double, 1> &result);

unsigned long int linearProgram2(const std::vector<ORCALine> &lines, double radius, const xt::xtensor<double, 1> &opt_velocity, bool direction_opt,
								 xt::xtensor<double, 1> &result);
#endif //MPPI_CA_CPP_ORCA_LINES_HPP
