#include "calibration/measurements.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numeric>

namespace calibration {
namespace {

constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();
constexpr double kInToCm = 2.54;
constexpr double kDegreesPerRev = 360;
constexpr double kTargetReserve = 0.80;
constexpr Field kTravelPerRev{"Known travel per motor rev (in/rev)", 0.1, 80, kUnset,
                              Remember::Travel};
constexpr Field kForwardPower{"Forward power (10-50)", 10, 50, 35};
constexpr Field kStaticPower{"Measured kS (power)", 0, 40, kUnset, Remember::StaticPower};
constexpr Field kVelocityGain{"Measured kV (power per in/s)", 0.1, 15, kUnset,
                              Remember::VelocityGain};
constexpr Field kPositionTolerance{"Web position tolerance (in)", 0.1, 2, kUnset,
                                   Remember::PositionTolerance};
constexpr Field kCorrection{"Desired correction power", 1, 127, 8};
constexpr Field kIntegralTime{"Integral time Ti (s, 0=off)", 0, 60, 0};
constexpr Field kDerivativeTime{"Derivative time Td (s, 0=off)", 0, 5, 0};

constexpr std::array kTravelBefore{Field{"Forward power (10-35)", 10, 35, 22}};
constexpr std::array kTravelAfter{Field{"Measured straight distance", 0.01, 300, kUnset},
                                  Field{"Distance units (1=in, 2=cm)", 1, 2, 1}};
constexpr std::array kSteadyBefore{kTravelPerRev, kForwardPower, kStaticPower};
constexpr std::array kAccelBefore{kTravelPerRev, kForwardPower, kStaticPower, kVelocityGain};
constexpr std::array kHeadingBefore{Field{"Right-turn power (10-35)", 10, 35, 25},
                                    Field{"Target clockwise turn (deg)", 1, 25, 12}};
constexpr std::array kSpeedBefore{kTravelPerRev, kForwardPower,
                                  Field{"Target speed (in/s)", 1, 50, 15, Remember::CruiseSpeed}};
constexpr std::array kPidAfter{kCorrection, kIntegralTime, kDerivativeTime};
constexpr std::array kStopBefore{kTravelPerRev, kForwardPower,
                                 Field{"Marked target distance (in)", 6, 36, 24}};
constexpr std::array kStopAfter{Field{"Largest measured stop error (in)", 0, 24, kUnset},
                                Field{"Position measurement margin (in)", 0, 2, 0.1},
                                Field{"Prior max heading error (deg)", 0, 90, 0},
                                Field{"Heading measurement margin (deg)", 0, 15, 1},
                                Field{"Web tracking limit (in, 0=skip)", 0, 8, 0}};
constexpr std::array kTrackingBefore{Field{"Max lateral error from video (in)", 0, 24, kUnset},
                                     Field{"Error measurement margin (in)", 0, 8, 0.1},
                                     Field{"Largest braking drift (in)", 0, 24, kUnset},
                                     Field{"Min swept-edge obstacle gap (in)", 0.01, 100, kUnset},
                                     kPositionTolerance};
constexpr std::array kLookaheadBefore{
    Field{"Representative speed (in/s)", 1, 50, kUnset, Remember::CruiseSpeed},
    Field{"Proposed preview time (s)", 0.01, 5, 0.25}};
constexpr std::array kRadiusBefore{Field{"Chord between arc ends (in)", 0.01, 200, kUnset},
                                   Field{"Arc midpoint rise over chord (in)", 0.01, 100, kUnset}};

constexpr std::array<Recipe, static_cast<std::size_t>(Test::Count)> kRecipes{{
    {"Travel per rev",
     {"Mark the chassis reference point on the mat.", "Run straight on the practice surface.",
      "Mark the same point again after the wheels stop.",
      "Measure between marks, then enter in or cm."},
     TrialKind::Travel,
     kTravelBefore,
     kTravelAfter},
    {"Static power kS",
     {"Use the same floor, tires and load as the route.",
      "The trial raises equal forward power slowly.",
      "It stops at the first sustained wheel movement.",
      "Repeat from rest and confirm it moved on the mat."},
     TrialKind::Breakaway,
     {},
     {}},
    {"Cruise kV / trim",
     {"Enter motor travel, forward power and measured kS.",
      "The trial records both sides at steady speed.",
      "Verify the tires gripped throughout the run.",
      "Cruise suggestion uses 80% of measured speed."},
     TrialKind::Steady,
     kSteadyBefore,
     {}},
    {"Acceleration kA",
     {"Enter motor travel and previously measured kS/kV.",
      "Start at rest on the same practice surface.",
      "The trial samples two early speeds at fixed power.",
      "Accel suggestion uses 80% of measured rise rate."},
     TrialKind::Accelerate,
     kAccelBefore,
     {}},
    {"Heading PID",
     {"Align the chassis and choose a target right turn.",
      "A timed right turn measures the resulting IMU angle.",
      "Enter a correction power and optional Ti / Td.",
      "These gains are starting values for a rerun."},
     TrialKind::Turn,
     kHeadingBefore,
     kPidAfter},
    {"Speed PID",
     {"Enter motor travel, trial power and target speed.",
      "The trial compares steady speed with the target.",
      "Enter a correction power and optional Ti / Td.",
      "These gains are starting values for a rerun."},
     TrialKind::Steady,
     kSpeedBefore,
     kPidAfter},
    {"Stop tolerances",
     {"Mark a target at the entered distance in inches.",
      "The trial brakes at the encoder-estimated target.",
      "Use a ruler for actual stop error after braking.",
      "Repeat and enter worst observed position / yaw."},
     TrialKind::StopAtDistance,
     kStopBefore,
     kStopAfter},
    {"Tracking / gap",
     {"Film repeated representative runs from above.",
      "Measure worst lateral error and braking drift.",
      "Measure obstacle gap from the swept chassis edge.",
      "Enter real measurements and position tolerance."},
     TrialKind::None,
     kTrackingBefore,
     {}},
    {"Lookahead",
     {"Use a speed repeatable on a broad practice arc.",
      "Enter that speed and a proposed preview time.",
      "Check the suggested distance on a second run.", "Revise if the chassis cuts the corner."},
     TrialKind::None,
     kLookaheadBefore,
     {}},
    {"Min turn radius",
     {"Mark the same chassis point on a repeatable arc.",
      "Choose a segment of at most half a circle.",
      "Measure its chord and midpoint rise, in inches.",
      "Verify clearance with the full chassis width."},
     TrialKind::None,
     kRadiusBefore,
     {}},
}};

struct BoundedValue {
    Value value;
    double min;
    double max;
    const char *error;
};

Result failure(const char *message) {
    return {.error = message};
}

Result output(std::initializer_list<BoundedValue> values) {
    Result result;
    for (const auto &entry : values) {
        if (!std::isfinite(entry.value.value) || entry.value.value < entry.min ||
            entry.value.value > entry.max) {
            return failure(entry.error);
        }
        result.values[result.count++] = entry.value;
    }
    return result;
}

bool valid_inputs(std::span<const Field> fields, std::span<const double> values) {
    if (fields.size() != values.size()) {
        return false;
    }
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (!std::isfinite(values[i]) || values[i] < fields[i].min || values[i] > fields[i].max) {
            return false;
        }
    }
    return true;
}

int command_power(double input_power) {
    return static_cast<int>(std::lround(input_power));
}

double mean_speed_in_s(const TrialResult &measured) {
    return (measured.left_in_s + measured.right_in_s) / 2;
}

bool steady_speeds_available(const TrialResult &measured) {
    return measured.left_in_s > 0 && measured.right_in_s > 0;
}

Result pid_estimate(Test test, double error, std::span<const double> settings) {
    if (!(error > 0)) {
        return failure("Error is zero. Choose a representative trial.");
    }
    const double kp = settings[0] / error;
    const double ki = settings[1] > 0 ? kp / settings[1] : 0;
    const double kd = kp * settings[2];
    if (test == Test::HeadingPid) {
        return output({{{"heading_p", kp, ""}, 0.01, 12, "Heading P out of range (0.01-12)."},
                       {{"heading_i", ki, ""}, 0, 10, "Heading I out of range (0-10)."},
                       {{"heading_d", kd, ""}, 0, 4, "Heading D out of range (0-4)."}});
    }
    return output({{{"speed_p", kp, ""}, 0, 15, "Speed P out of range (0-15)."},
                   {{"speed_i", ki, ""}, 0, 10, "Speed I out of range (0-10)."},
                   {{"speed_d", kd, ""}, 0, 5, "Speed D out of range (0-5)."}});
}

} // namespace

const Recipe &recipe(Test test) {
    return kRecipes[static_cast<std::size_t>(test)];
}

TrialConfig trial_config(Test test, std::span<const double> before) {
    const auto &item = recipe(test);
    if (!valid_inputs(item.before, before)) {
        return {};
    }
    switch (item.trial) {
    case TrialKind::None:
    case TrialKind::Breakaway:
        return {item.trial};
    case TrialKind::Travel:
    case TrialKind::Turn:
        return {item.trial, 0, command_power(before[0])};
    case TrialKind::Steady:
    case TrialKind::Accelerate:
        return {item.trial, before[0], command_power(before[1])};
    case TrialKind::StopAtDistance:
        return {item.trial, before[0], command_power(before[1]), before[2]};
    }
    return {};
}

Result calculate(Test test, std::span<const double> before, std::span<const double> after,
                 const TrialResult &measured) {
    const auto &item = recipe(test);
    if (!valid_inputs(item.before, before) || !valid_inputs(item.after, after)) {
        return failure("Enter each input within its displayed range.");
    }
    switch (test) {
    case Test::Travel: {
        if (after[1] != 1 && after[1] != 2) {
            return failure("Choose distance units 1 (in) or 2 (cm).");
        }
        const double abs_deg = std::accumulate(
            measured.delta_deg.begin(), measured.delta_deg.end(), 0.0,
            [](double total, double delta_deg) { return total + std::abs(delta_deg); });
        if (abs_deg <= 0) {
            return failure("No encoder travel. Repeat the straight run.");
        }
        const double measured_in = after[1] == 2 ? after[0] / kInToCm : after[0];
        const double mean_rev = abs_deg / (measured.delta_deg.size() * kDegreesPerRev);
        return output({{{"travel_per_rev", measured_in / mean_rev, "in/rev", Remember::Travel},
                        0.1,
                        80,
                        "Travel out of range (0.1-80 in/rev). Check units."}});
    }
    case Test::StaticPower:
        return output({{{"static_power", static_cast<double>(measured.breakaway_power), "power",
                         Remember::StaticPower},
                        1,
                        40,
                        "No breakaway found within the kS range."}});
    case Test::Steady: {
        const double net_power = command_power(before[1]) - before[2];
        if (net_power <= 0 || !steady_speeds_available(measured)) {
            return failure("Need power above kS and both sides moving forward.");
        }
        const double speed_in_s = mean_speed_in_s(measured);
        const double imbalance = (measured.right_in_s - measured.left_in_s) / (2 * speed_in_s);
        return output(
            {{{"velocity_gain", net_power / speed_in_s, "", Remember::VelocityGain},
              0.1,
              15,
              "kV out of range (0.1-15). Check travel and kS."},
             {{"left_trim", net_power * imbalance, "power"},
              -15,
              15,
              "Trim out of range (+/-15). Inspect the drivetrain."},
             {{"cruise_speed", kTargetReserve * speed_in_s, "in/s", Remember::CruiseSpeed},
              1,
              50,
              "Cruise out of range (1-50 in/s). Check speed."}});
    }
    case Test::Accelerate: {
        const double change_in_s = measured.accel_end_in_s - measured.accel_start_in_s;
        if (change_in_s <= 0 || measured.accel_time_s <= 0 || measured.accel_start_in_s < 0) {
            return failure("Need two forward speeds during acceleration.");
        }
        const double accel_in_s2 = change_in_s / measured.accel_time_s;
        const double mean_in_s = (measured.accel_start_in_s + measured.accel_end_in_s) / 2;
        const double accel_power = command_power(before[1]) - before[2] - before[3] * mean_in_s;
        if (accel_power < 0) {
            return failure("No power left for acceleration. Check kS and kV.");
        }
        return output({{{"acceleration_gain", accel_power / accel_in_s2, ""},
                        0,
                        5,
                        "kA out of range (0-5). Verify the speed samples."},
                       {{"acceleration", kTargetReserve * accel_in_s2, "in/s2"},
                        1,
                        150,
                        "Acceleration out of range (1-150 in/s2)."}});
    }
    case Test::HeadingPid:
        if (!(measured.yaw_deg > 0)) {
            return failure("No clockwise turn measured. Check IMU and motion.");
        }
        return pid_estimate(test, std::abs(before[1] - measured.yaw_deg), after);
    case Test::SpeedPid:
        if (!steady_speeds_available(measured)) {
            return failure("Need forward steady speed from both sides.");
        }
        return pid_estimate(test, std::abs(before[2] - mean_speed_in_s(measured)), after);
    case Test::Stop: {
        const double position_in = after[0] + after[1];
        const double heading_deg = std::max(std::abs(measured.yaw_deg), after[2]) + after[3];
        if (after[4] > 0 && position_in >= after[4]) {
            return failure("Position tolerance must be below tracking limit.");
        }
        return output({{{"position_tolerance", position_in, "in", Remember::PositionTolerance},
                        0.1,
                        2,
                        "Position tolerance out of range (0.1-2 in)."},
                       {{"heading_tolerance", heading_deg, "deg"},
                        1,
                        15,
                        "Heading tolerance out of range (1-15 deg)."}});
    }
    case Test::Tracking: {
        const double tracking_in = before[0] + before[1];
        const double clearance_in = tracking_in + before[2];
        if (tracking_in <= before[4]) {
            return failure("Tracking limit must exceed position tolerance.");
        }
        if (clearance_in >= before[3]) {
            return failure("Tracking plus braking drift uses up the gap.");
        }
        return output(
            {{{"tracking_limit", tracking_in, "in"},
              0.5,
              8,
              "Tracking limit out of range (0.5-8 in)."},
             {{"clearance", clearance_in, "in"}, 0, 4, "Clearance out of range (0-4 in)."}});
    }
    case Test::Lookahead:
        return output({{{"lookahead", before[0] * before[1], "in"},
                        1,
                        18,
                        "Lookahead out of range (1-18 in)."}});
    case Test::Radius: {
        const double chord_in = before[0];
        const double rise_in = before[1];
        if (rise_in > chord_in / 2) {
            return failure("Rise exceeds half the chord. Use a shorter arc.");
        }
        const double radius_in = chord_in * chord_in / (8 * rise_in) + rise_in / 2;
        return output({{{"min_radius", radius_in, "in"},
                        0.25,
                        24,
                        "Turn radius out of range (0.25-24 in)."}});
    }
    case Test::Count:
        break;
    }
    return failure("Select a measurement project.");
}

} // namespace calibration
