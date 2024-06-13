#include "dynamics.hpp"


auto DiffDrive::shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1> {
	return xt::zeros<double>({DD_STATE_DIM});
}

auto DiffDrive::dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
						double dt) -> xt::xtensor<double, 1> {
	xt::xtensor<double, 1> result = xt::empty<double>({DD_STATE_DIM});
	double theta = state[2];
	double v = control[0];
	double w = control[1];
	result[0] = v * std::cos(theta) * dt;
	result[1] = v * std::sin(theta) * dt;
	result[2] = w * dt;
	return result;
}


auto CarLike::shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1> {
	return xt::zeros<double>({DD_STATE_DIM});
}


auto CarLike::dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
					  double dt) -> xt::xtensor<double, 1> {

	double in_L = 1.0/dist_between_axles;
	xt::xtensor<double, 1> result = xt::empty<double>({DD_STATE_DIM});
	double theta = state[2];
	double v = control[0];
	double phi = control[1];
	result[0] = v * std::cos(theta) * dt;
	result[1] = v * std::sin(theta) * dt;
	result[2] = (v * in_L *  std::tan(phi)) * dt;
	return result;
}
