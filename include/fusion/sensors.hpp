#pragma once
// Simulated radar and camera. Both report polar measurements in the ego frame
// (x forward, y left), with the error profiles that make fusion worthwhile:
// radar measures range and range rate well but bearing poorly, while a mono
// camera measures bearing well but range poorly, and worse with distance.
#include <random>
#include <vector>

namespace fusion {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCarLength = 4.5;
constexpr double kCarWidth = 1.8;

struct Truth {
    int id;
    double x, y, vx, vy;  // relative to the ego
};

enum class SensorKind { Radar, Camera };

struct SensorSpec {
    SensorKind kind;
    double rate_hz;
    double half_fov;          // rad
    double min_range, max_range;
    double p_detect;
    double clutter_mean;      // false detections per scan
    double sigma_range_const, sigma_range_lin, sigma_range_quad;  // c + l*r + q*r^2
    double sigma_bearing;     // rad
    double sigma_range_rate;  // m/s, radar only
    // Detection probability for a car that is mostly hidden behind a nearer one.
    // Radar still gets some returns that bounce under the car in front; a camera
    // gets none.
    double p_detect_occluded;
};

SensorSpec radar_spec();
SensorSpec camera_spec();

struct Detection {
    SensorKind kind;
    double t;
    double range, bearing, range_rate;  // range_rate is 0 for the camera
    int truth_id;  // -1 for clutter. Used only for evaluation, never by the tracker.
};

double range_sigma(const SensorSpec& s, double range);
bool in_view(const SensorSpec& s, double x, double y);

// Fraction of the target's angular width, seen from the ego, that no nearer car
// covers. Cars are boxes of kCarLength by kCarWidth aligned with the road.
double visible_fraction(double x, double y, const std::vector<Truth>& others);
constexpr double kOccludedBelow = 0.5;  // a car less than half visible counts as hidden

// With occlusion on, a car less than half visible is detected with
// p_detect_occluded instead of p_detect.
std::vector<Detection> sense(const SensorSpec& s, const std::vector<Truth>& truths, double t,
                             std::mt19937_64& rng, bool occlusion = false);

}  // namespace fusion
