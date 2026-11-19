#pragma once

#include "autonomous/route.hpp"

#include <cstddef>
#include <vector>

namespace auto_route {

struct Sample {
    Point position;
    double distance_in;
    double heading_deg;
};

double distance_in(Point from, Point to);
double bearing_deg(Point from, Point to);
double angle_error_deg(double target_deg, double current_deg);

// 将三阶 Bézier 离散为按弧长标记的坐标和差速底盘的切线航向。
std::vector<Sample> sample_curve(Pose from, const Step &step);
Point lookahead_point(const std::vector<Sample> &path, std::size_t nearest, double ahead_in);
std::size_t advance_nearest(const std::vector<Sample> &path, std::size_t nearest, Point current,
                            double search_in);

} // namespace auto_route
