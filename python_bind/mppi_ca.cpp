#include <pybind11/pybind11.h>
#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pytensor.hpp>
#include "../include/controller.hpp"

namespace py = pybind11;

PYBIND11_MODULE(mppi_ca, m) {
	xt::import_numpy();
	py::class_<mppica::Controller>(m, "Controller")
			.def(py::init<>())
			.def("next_step", &mppica::Controller::nextStep);
}


//PYBIND11_MODULE(bind_module, m) {
//	xt::import_numpy();
//	py::class_<Controller>(m, "Controller")
//			.def(py::init<>())
//			.def("next_step", &Controller::nextStep);
//}


