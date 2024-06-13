

#ifndef MPPI_CA_CPP_PYTHON_CONTROLLER_H
#define MPPI_CA_CPP_PYTHON_CONTROLLER_H
#include <stdexcept>

#include "../include/controller.hpp"
#include <pybind11/pybind11.h>
#include "xtensor-python/pyarray.hpp"

namespace py = pybind11;

mppica::AgentParams convertAgentParams(const py::dict& params_dict);
mppica::MPPIParams convertAlgParams(const py::dict& params_dict);

// py::object getValue(const py::dict& data, const std::string &key, const py::object &default_value);

template <typename T>
T getValue(const py::dict& data, const std::string &key, const T &default_value);



#endif //MPPI_CA_CPP_PYTHON_CONTROLLER_H
