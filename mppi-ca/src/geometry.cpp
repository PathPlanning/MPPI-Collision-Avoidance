#include "geometry.hpp"

double pointToLineSegmentDistance(const xt::xtensor<double, 1> &line_p1,
								  const xt::xtensor<double, 1> &line_p2,
								  const xt::xtensor<double, 1> &point) {
	if (xt::allclose(line_p1, point) or xt::allclose(line_p2, point))
		return 0.0;

	auto v = line_p2 - line_p1;
	auto w = point - line_p1;
	double c1, c2;

	c1 = xt::linalg::dot(w, v)[0];

	if (c1 <= 0.0)
		return xt::linalg::norm(w, 2);
	c2 = xt::linalg::dot(v, v)[0];
	if (c2 <= c1)
		return xt::linalg::norm(point - line_p2, 2);

	double b = c1 / c2;
	auto pb = line_p1 + v * b;
	return xt::linalg::norm(point - pb, 2);
}

double determinant2D(const xt::xtensor<double, 1> &ab, const xt::xtensor<double, 1> &cd) {
	return ab[0] * cd[1] - ab[1] * cd[0];
}


std::tuple<bool, xt::xtensor<double, 1>> lineIntersection(const xt::xtensor<double, 1> &line1, const xt::xtensor<double, 1> &line2) {
	double a1 = line1[0], b1 = line1[1], c1 = line1[2];

	double a2 = line2[0], b2 = line2[1], c2 = line2[2];
	double determinant = a1*b2 - a2*b1;

	if (determinant == 0)
		return {false, xt::zeros<double>({0})};
	else {
		double x = (b2 * -c1 - b1 * -c2) / determinant;
		double y = (a1 * -c2 - a2 * -c1) / determinant;
		return {true, xt::xtensor<double, 1>({x, y})};
	}
}