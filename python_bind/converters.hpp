#include "../include/controller.hpp"
#include <pybind11/pybind11.h>
#include "xtensor-python/pyarray.hpp"

#ifndef MPPI_CA_CPP_PYTHON_CONTROLLER_H
#define MPPI_CA_CPP_PYTHON_CONTROLLER_H

namespace py = pybind11;

mppica::AgentParams convertAgentParams(const py::dict& params_dict);
mppica::MPPIParams convertAlgParams(const py::dict& params_dict);



#endif //MPPI_CA_CPP_PYTHON_CONTROLLER_H
