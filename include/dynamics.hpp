#include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xfixed.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xrandom.hpp>
#include <iostream>
#include <cmath>

#include "consts.hpp"

#ifndef MPPI_CA_CPP_DIFF_DRIVE_HPP
#define MPPI_CA_CPP_DIFF_DRIVE_HPP


class DiffDrive {
	public:
		auto shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1>;

		auto dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
					 double dt) -> xt::xtensor<double, 1>;


};


class CarLike {
	public:
		auto shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1>;

		auto dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
					 double dt) -> xt::xtensor<double, 1>;

		double dist_between_axles;
};


#endif //MPPI_CA_CPP_DIFF_DRIVE_HPP
