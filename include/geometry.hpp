#include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xfixed.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <cmath>

#ifndef MPPI_CA_CPP_GEOMETRY_HPP
#define MPPI_CA_CPP_GEOMETRY_HPP

double point_to_line_segment_distance(const xt::xtensor<double, 1> &line_p1,
									  const xt::xtensor<double, 1> &line_p2,
									  const xt::xtensor<double, 1> &point);


#endif //MPPI_CA_CPP_GEOMETRY_HPP
