#pragma once
// Scores confirmed tracks against ground truth inside a region that both
// sensors cover (5-80 m ahead, within 28 degrees), so every sensor setup is
// judged on the same area. Tracks are matched to all cars, so a correct track
// of a car just outside the region is not counted as false.
#include <array>
#include <map>
#include <vector>

#include "fusion/sensors.hpp"
#include "fusion/tracker.hpp"

namespace fusion {

// A track counts as following a car if it is within 4 m along the road and
// 1.5 m across it. Further across would put it in the neighbouring lane.
constexpr double kMatchLon = 4.0;
constexpr double kMatchLat = 1.5;
constexpr std::array<double, 4> kRangeBins = {5.0, 30.0, 55.0, 80.0};

bool in_region(double x, double y);
int lane_of(double y);

struct Totals {
    double sq_lon = 0, sq_lat = 0, sq_vel = 0, nees = 0;
    long matched = 0, truths = 0, lane_correct = 0, false_tracks = 0, id_switches = 0, steps = 0;
    std::array<double, 3> sq_lat_bin{};
    std::array<long, 3> n_bin{};
    double seconds = 0;
    double delay_sum = 0;  // time from entering the region to first being tracked
    long delay_n = 0;
    void add(const Totals& o);
};

struct Summary {
    double rmse_lon, rmse_lat, rmse_vel, mean_nees;
    double lane_accuracy, coverage, false_tracks, id_switches_per_min;
    double time_to_track;  // mean seconds a car is in the region before it has a track
    std::array<double, 3> rmse_lat_by_range;
};

Summary summarize(const Totals& t);

class Evaluator {
public:
    void add(const std::vector<Truth>& truths, const std::vector<Track>& tracks, double dt);
    const Totals& totals() const { return totals_; }

private:
    Totals totals_;
    std::map<int, int> last_track_;  // truth id -> track id it was last matched to
    std::map<int, double> waiting_;  // truth id -> time it entered the region untracked
    std::map<int, bool> seen_;       // truth ids already scored for time to track
    double now_ = 0;
};

}  // namespace fusion
