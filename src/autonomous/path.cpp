#include "autonomous/path.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace auto_route {
namespace {

Point interpolate(Point a, Point b, double t) {
    return {std::lerp(a.x, b.x, t), std::lerp(a.y, b.y, t)};
}

Point bezier(Point start, const Step &step, double t) {
    const auto a = interpolate(start, step.control1, t);
    const auto b = interpolate(step.control1, step.control2, t);
    const auto c = interpolate(step.control2, step.end, t);
    return interpolate(interpolate(a, b, t), interpolate(b, c, t), t);
}

} // namespace

double distance_in(Point from, Point to) {
    return std::hypot(to.x - from.x, to.y - from.y);
}

double bearing_deg(Point from, Point to) {
    return std::atan2(to.x - from.x, to.y - from.y) * 180.0 / std::numbers::pi;
}

double angle_error_deg(double target_deg, double current_deg) {
    return std::remainder(target_deg - current_deg, 360.0);
}

std::vector<Sample> sample_curve(Pose from, const Step &step) {
    const double span_in = distance_in(from.position, step.control1) +
                           distance_in(step.control1, step.control2) +
                           distance_in(step.control2, step.end);
    const int count = std::max(16, static_cast<int>(std::ceil(span_in * 4.0)));
    std::vector<Sample> points;
    points.reserve(count + 1);
    points.push_back({from.position, 0.0, from.heading_deg});
    for (int index = 1; index <= count; ++index) {
        const Point point = bezier(from.position, step, static_cast<double>(index) / count);
        const double stride_in = distance_in(points.back().position, point);
        if (stride_in > 0.00001) {
            points.push_back({point, points.back().distance_in + stride_in, 0.0});
        }
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto before = points[index == 0 ? 0 : index - 1].position;
        const auto after = points[std::min(points.size() - 1, index + 1)].position;
        points[index].heading_deg = bearing_deg(before, after) + (step.reverse ? 180.0 : 0.0);
    }
    return points;
}

Point lookahead_point(const std::vector<Sample> &path, std::size_t nearest, double ahead_in) {
    const double target_in = path[nearest].distance_in + ahead_in;
    const auto end = std::lower_bound(
        path.begin() + nearest, path.end(), target_in,
        [](const Sample &point, double value) { return point.distance_in < value; });
    if (end == path.end()) {
        return path.back().position;
    }
    const auto prior = end - 1;
    return interpolate(prior->position, end->position,
                       (target_in - prior->distance_in) / (end->distance_in - prior->distance_in));
}

std::size_t advance_nearest(const std::vector<Sample> &path, std::size_t nearest, Point current,
                            double search_in) {
    const double max_in = path[nearest].distance_in + search_in;
    double best_in = distance_in(current, path[nearest].position);
    for (auto index = nearest + 1; index < path.size() && path[index].distance_in <= max_in;
         ++index) {
        const double candidate_in = distance_in(current, path[index].position);
        if (candidate_in < best_in) {
            best_in = candidate_in;
            nearest = index;
        }
    }
    return nearest;
}

} // namespace auto_route
