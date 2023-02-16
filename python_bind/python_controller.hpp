#include <xtensor-python/pytensor.hpp>
#include "../include/controller.hpp"

#ifndef MPPI_CA_CPP_PYTHON_CONTROLLER_H
#define MPPI_CA_CPP_PYTHON_CONTROLLER_H

class Controller
{
	public:
		Controller();
		auto nextStep(const xt::pytensor<double, 1> &curr_state, const  xt::pytensor<double, 2> &neighbours_states,
					  const xt::pytensor<double, 1> &goal_state) -> xt::pytensor<double, 1>;
		void reset();

	private:
		mppica::Controller mppi_controller;
};





#endif //MPPI_CA_CPP_PYTHON_CONTROLLER_H
