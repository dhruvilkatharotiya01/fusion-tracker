#pragma once
// Multi-target tracker: EKF per track, gating, global nearest neighbour
// association, and M-of-N track confirmation. Radar and camera scans arrive at
// different rates and are processed in time order as they come in.
#include <vector>

#include "fusion/ekf.hpp"
#include "fusion/sensors.hpp"

namespace fusion {

struct TrackerConfig {
    MotionNoise motion;
    int confirm_hits = 3;      // confirm after this many hits ...
    int confirm_window = 5;    // ... within this many scans that could have seen the track
    double coast_time = 1.0;   // delete after this long without an update, s
    double max_position_sigma = 8.0;  // delete once the position is this uncertain, m
    // Delete the younger of two tracks whose positions are statistically the
    // same. Without this, a track started by radar clutter next to a real car
    // can live on camera detections, which carry no velocity information.
    bool prune_duplicates = true;
    // Count hits and misses for confirmation only from the sensor that started
    // the track. Otherwise a camera detection of a real car, whose range is
    // uncertain, can confirm a track that radar clutter started nearby.
    bool confirm_by_source = true;
    // When a missed track is predicted to be hidden behind a nearer confirmed
    // track, do not count the miss and keep it for up to occluded_coast_time.
    bool occlusion_coasting = false;
    double occluded_coast_time = 3.0;  // s
};

struct Track {
    int id;
    State x;
    Cov P;
    double t;            // time of x and P
    double last_update;
    bool confirmed = false;
    std::vector<bool> history;  // hit or miss for recent scans that covered the track
    SensorKind source = SensorKind::Radar;  // sensor whose detection started the track
    bool occluded = false;  // predicted to be hidden behind a nearer track at the last miss
};

// Gates: 99.5% points of the chi-square distribution.
constexpr double kDuplicateGate = 9.21;  // 99% point, 2 degrees of freedom (position only)
constexpr double kGate2 = 10.597;  // 2 degrees of freedom (camera)
constexpr double kGate3 = 12.838;  // 3 degrees of freedom (radar)

class Tracker {
public:
    explicit Tracker(TrackerConfig cfg = {});
    void process(const std::vector<Detection>& scan, const SensorSpec& sensor, double t);
    std::vector<Track> confirmed_at(double t) const;  // confirmed tracks predicted to t
    const std::vector<Track>& tracks() const { return tracks_; }

private:
    void record(Track& track, bool hit) const;
    bool expired(const Track& track, double t) const;
    void prune_duplicates();
    bool hidden_by_others(std::size_t i) const;

    TrackerConfig cfg_;
    std::vector<Track> tracks_;
    int next_id_ = 1;
};

}  // namespace fusion
