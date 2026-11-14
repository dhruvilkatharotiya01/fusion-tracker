#pragma once
// Three-lane highway traffic around an ego car driving at constant speed in the
// middle lane. Other cars follow the car ahead with the Intelligent Driver
// Model, change their preferred speed now and then, and change lanes when the
// target lane has a gap. New cars enter as old ones leave, so the tracker has
// to start and end tracks all the time.
#include <cstdint>
#include <random>
#include <vector>

#include "fusion/sensors.hpp"

namespace fusion {

constexpr double kLaneWidth = 3.5;

class Traffic {
public:
    explicit Traffic(std::uint64_t seed, int cars = 12, double ego_speed = 30.0);
    void step(double dt);
    std::vector<Truth> truths() const;
    double time() const { return t_; }

private:
    struct Car {
        int id;
        double x, y, vy, speed;
        double desired_speed, next_speed_change;
        int lane, target_lane;
        double lane_start, lane_duration;
    };
    void place(Car& car, bool initial);
    bool crowded(const Car& car) const;
    bool lane_free(const Car& car, int lane) const;
    double idm_accel(const Car& car) const;

    std::mt19937_64 rng_;
    double ego_speed_;
    double t_ = 0.0;
    int next_id_ = 0;
    std::vector<Car> cars_;
};

}  // namespace fusion
