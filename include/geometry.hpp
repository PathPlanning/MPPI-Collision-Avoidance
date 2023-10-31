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

double pointToLineSegmentDistance(const xt::xtensor<double, 1> &line_p1,
								  const xt::xtensor<double, 1> &line_p2,
								  const xt::xtensor<double, 1> &point);

double determinant2D(const xt::xtensor<double, 1> &ab, const xt::xtensor<double, 1> &cd);


std::tuple<bool, xt::xtensor<double, 1>> lineIntersection(const xt::xtensor<double, 1> &line1, const xt::xtensor<double, 1> &line2);


#endif //MPPI_CA_CPP_GEOMETRY_HPP
