#include "orca_lines.hpp"

std::vector<ORCALine>
computeORCALines(ORCAParams params, xt::xtensor<double, 1> agent_state, xt::xtensor<double, 2> neighbors_states,
				 xt::xtensor<double, 1> neighbors_sizes) {
	std::vector<ORCALine> lines;
	ORCALine curr_line;
	double inv_time_boundary = 1 / params.time_boundary;
	xt::xtensor<double, 1> u, w, nw;
	double w_length;
	size_t neighbors_num = (params.agents_max_num < neighbors_states.shape(0)) ? params.agents_max_num
																			   : neighbors_states.shape(0);

	bool optimize_zero_velocities = params.opt_zero_vel;
	xt::xtensor<double, 1> agent_pos = {agent_state[0], agent_state[1]};
	xt::xtensor<double, 1> agent_vel = {agent_state[3], agent_state[4]};
	xt::xtensor<double, 1> neighbor_pos = xt::zeros<double>({2});
	xt::xtensor<double, 1> neighbor_vel = xt::zeros<double>({2});
	xt::xtensor<double, 1> rel_position, rel_velocity;
	double radius_sum, radius_sum_sq, curr_dist, curr_dist_sq;

	for (size_t neighbor = 0; neighbor < neighbors_num; ++neighbor) {
		neighbor_pos = xt::view(neighbors_states, neighbor, xt::range(0, 2));
		neighbor_vel = xt::view(neighbors_states, neighbor, xt::range(2, 4));

		rel_position = neighbor_pos - agent_pos; // (P_b - P_a)
		curr_dist = xt::linalg::norm(rel_position, 2);
		if (curr_dist > params.ignore_radius) {
			continue;
		}

		if (optimize_zero_velocities)
			rel_velocity = xt::zeros<double>({2}); // (0, 0)
		else
			rel_velocity = (agent_vel - neighbor_vel); // (V_a - V_b)

		radius_sum = params.agent_size + neighbors_sizes[neighbor]; // (R_a + R_b)
		radius_sum_sq = radius_sum * radius_sum;

		curr_dist_sq = curr_dist * curr_dist;

		if (curr_dist >= radius_sum) {
			w = rel_velocity - (rel_position * inv_time_boundary);

			w_length = xt::linalg::norm(w, 2);

			double sq_w_length = w_length * w_length;
			double w_proj = xt::linalg::dot(w, rel_position)[0];


			if (w_proj < 0.0f and (w_proj * w_proj) > sq_w_length * radius_sum_sq) {
				nw = w / w_length;
				curr_line.dir[0] = nw[1];
				curr_line.dir[1] = -nw[0];
				u = nw * ((radius_sum * inv_time_boundary) - w_length);
			}
			else {
				double leg = std::sqrt(curr_dist_sq - radius_sum_sq);
				if (determinant2D(rel_position, w) > 0.0f) {
					curr_line.dir[0] = rel_position[0] * leg - rel_position[1] * radius_sum;
					curr_line.dir[1] = rel_position[0] * radius_sum + rel_position[1] * leg;
					curr_line.dir /= curr_dist_sq;
				}
				else {
					curr_line.dir[0] = rel_position[0] * leg + rel_position[1] * radius_sum;
					curr_line.dir[1] = -rel_position[0] * radius_sum + rel_position[1] * leg;
					curr_line.dir *= -1;
					curr_line.dir /= curr_dist_sq;
				}
				double rv_proj = xt::linalg::dot(rel_velocity, curr_line.dir)[0];
				u = curr_line.dir * rv_proj - rel_velocity;
			}
		}
		else {
			double inv_time_step = 1.0f / params.time_step;
			w = rel_velocity - (rel_position * inv_time_step);
			w_length = xt::linalg::norm(w, 2);
			nw = w / w_length;
			curr_line.dir[0] = nw[1];
			curr_line.dir[1] = -nw[0];
			u = nw * ((radius_sum * inv_time_step) - w_length);
		}

		if (optimize_zero_velocities)
			curr_line.point_lies_on = u * params.responsibility_factor;
		else
			curr_line.point_lies_on = agent_vel + (u * params.responsibility_factor);

		lines.push_back(curr_line);
	}
	return lines;
}

xt::xtensor<double, 2>
computeORCALinesXT(size_t agents_max_num, double agent_size, double time_boundary, double time_step,
				   double responsibility_factor, bool opt_zero_vel, xt::xtensor<double, 1> agent_state,
				   xt::xtensor<double, 2> neighbors_states, xt::xtensor<double, 1> neighbors_sizes) {

	auto params = ORCAParams();
	params.agents_max_num = agents_max_num;
	params.agent_size = agent_size;
	params.time_boundary = time_boundary;
	params.time_step = time_step;
	params.responsibility_factor = responsibility_factor;
	params.opt_zero_vel = opt_zero_vel;

	auto orca_lines = computeORCALines(params, agent_state, neighbors_states, neighbors_sizes);
	xt::xtensor<double, 2> lines = xt::zeros<double>({orca_lines.size(), static_cast<size_t>(4)});

	for (size_t line = 0; line < orca_lines.size(); ++line) {
		lines[{line, static_cast<size_t>(0)}] = orca_lines[line].dir[0];
		lines[{line, static_cast<size_t>(1)}] = orca_lines[line].dir[1];
		lines[{line, static_cast<size_t>(2)}] = orca_lines[line].point_lies_on[0];
		lines[{line, static_cast<size_t>(3)}] = orca_lines[line].point_lies_on[1];
	}
	return lines;
}


bool linearProgram1(const std::vector<ORCALine> &lines, unsigned long curr, float radius,
					const xt::xtensor<double, 1> &opt_velocity,
					bool direction_opt, xt::xtensor<double, 1> &result) {
	double dot_product = xt::linalg::dot(lines[curr].point_lies_on, lines[curr].dir)[0];
	float discriminant = dot_product * dot_product + radius * radius - pow(xt::linalg::norm(lines[curr].point_lies_on), 2);

	if (discriminant < 0.0f) {
		return false;
	}

	double sqrt_discriminant = std::sqrt(discriminant);
	double t_left = -dot_product - sqrt_discriminant;
	double t_right = -dot_product + sqrt_discriminant;

	for (int i = 0; i < curr; ++i) {

		const double denominator = determinant2D(lines[curr].dir, lines[i].dir);
		const double numerator = determinant2D(lines[i].dir, lines[curr].point_lies_on - lines[i].point_lies_on);

		if (std::fabs(denominator) <= 0.00001) {
			if (numerator < 0.0f) {
				return false;
			}
			else {
				continue;
			}
		}

		const double t = numerator / denominator;

		if (denominator >= 0.0f) {
			t_right = std::min(t_right, t);
		}
		else {
			t_left = std::max(t_left, t);
		}

		if (t_left > t_right) {
			return false;
		}
	}

	if (direction_opt) {
		if (xt::linalg::dot(opt_velocity, lines[curr].dir)[0] > 0.0f) {
			result = lines[curr].point_lies_on + lines[curr].dir * t_right;
		}
		else {
			result = lines[curr].point_lies_on + lines[curr].dir * t_left;
		}
	}
	else {
		const double t = xt::linalg::dot(lines[curr].dir, opt_velocity - lines[curr].point_lies_on)[0];

		if (t < t_left) {
			result = lines[curr].point_lies_on + lines[curr].dir * t_left;
		}
		else if (t > t_right) {
			result = lines[curr].point_lies_on + lines[curr].dir * t_right;
		}
		else {
			result = lines[curr].point_lies_on + lines[curr].dir * t;
		}
	}

	return true;
}


unsigned long int
linearProgram2(const std::vector<ORCALine> &lines, double radius, const xt::xtensor<double, 1> &opt_velocity,
			   bool direction_opt, xt::xtensor<double, 1> &result) {
	if (direction_opt) {
		result = opt_velocity * radius;
	}
	else if (xt::linalg::norm(opt_velocity) > radius) {
		result = (opt_velocity / xt::linalg::norm(opt_velocity)) * radius;
	}
	else {
		result = opt_velocity;
	}

	for (unsigned long int i = 0; i < lines.size(); ++i) {

		if (determinant2D(lines[i].dir, lines[i].point_lies_on - result) > 0.0f) {
			const xt::xtensor<double, 1> temp_result = result;

			if (!linearProgram1(lines, i, radius, opt_velocity, direction_opt, result)) {
				result = temp_result;
				return i;
			}
		}
	}
	return lines.size();
}
