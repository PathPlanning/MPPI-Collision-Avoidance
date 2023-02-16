#include "python_controller.hpp"

Controller::Controller()
{
	mppi_controller = mppica::Controller();
}

auto Controller::nextStep(const xt::pytensor<double, 1> &curr_state, const xt::pytensor<double, 2> &neighbours_states,
						  const xt::pytensor<double, 1> &goal_state) -> xt::pytensor<double, 1>
{
//	xt::xtensor<double, 1> cs = curr_state;
//	xt::xtensor<double, 2> ns = neighbours_states;
//	xt::xtensor<double, 1> gs = goal_state;
	std::cout << "AAA\n";

//	xt::xtensor<double, 1> cs = xt::ones<double>({3});
//	xt::xtensor<double, 1> gs= xt::ones<double>({3});;
//	xt::xtensor<double, 2> ns = xt::ones<double>({10,4});

	return mppi_controller.nextStep(curr_state, neighbours_states, goal_state);

}


void Controller::reset()
{
	mppi_controller.reset();
}

