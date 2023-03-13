#include <pybind11/pybind11.h>
#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pytensor.hpp>
#include "../include/controller.hpp"
#include "converters.hpp"

namespace py = pybind11;

PYBIND11_MODULE(mppi_ca_cpp, m) {
	xt::import_numpy();
	py::class_<mppica::AgentParams>(m, "AgentParams")
			.def(py::init<>());
	py::class_<mppica::MPPIParams>(m, "AlgParams")
			.def(py::init<>());
	py::class_<mppica::Controller>(m, "Controller")
			.def(py::init<>())
			.def("next_step", &mppica::Controller::nextStep)
			.def("set_agent_params", &mppica::Controller::setAgentParams)
			.def("set_alg_params", &mppica::Controller::setAlgParams);

	m.def("convert_agent_params", &convertAgentParams, "Coverts agent params");
	m.def("convert_alg_params", &convertAlgParams, "Coverts alg parameters");
}


//PYBIND11_MODULE(bind_module, m) {
//	xt::import_numpy();
//	py::class_<Controller>(m, "Controller")
//			.def(py::init<>())
//			.def("next_step", &Controller::nextStep);
//}


