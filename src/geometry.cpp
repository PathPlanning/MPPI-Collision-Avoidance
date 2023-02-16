#include "geometry.hpp"

double point_to_line_segment_distance(const xt::xtensor<double, 1> &line_p1,
									  const xt::xtensor<double, 1> &line_p2,
									  const xt::xtensor<double, 1> &point)
{
	if (xt::allclose(line_p1, point) or xt::allclose(line_p2, point))
		return 0.0;

	auto v = line_p2 - line_p1;
	auto w = point - line_p1;
	double c1, c2;

	c1 = xt::linalg::dot( w, v)[0];

	if (c1 <= 0.0)
		return xt::linalg::norm(w, 2);
	c2 = xt::linalg::dot( v, v)[0];
	if (c2 <= c1)
		return xt::linalg::norm(point - line_p2, 2);

	double b = c1 / c2;
	auto pb = line_p1 + v * b;
	return xt::linalg::norm(point - pb, 2);
}
