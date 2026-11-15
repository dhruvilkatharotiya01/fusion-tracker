#include "fusion/traffic.hpp"

#include <algorithm>
#include <cmath>

namespace fusion {

namespace {

double uniform(std::mt19937_64& rng, double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
}


// A car occupies its lane, and also the lane it is moving into.
bool occupies(int lane, int target_lane, int which) { return lane == which || target_lane == which; }

}  // namespace

Traffic::Traffic(std::uint64_t seed, int cars, double ego_speed) : rng_(seed), ego_speed_(ego_speed) {
    cars_.resize(cars);
    for (Car& car : cars_) place(car, true);
}

bool Traffic::crowded(const Car& car) const {
    if (car.lane == 0 && std::abs(car.x) < 15.0) return true;  // keep clear of the ego
    for (const Car& other : cars_)
        if (&other != &car && occupies(other.lane, other.target_lane, car.lane) && std::abs(other.x - car.x) < 30.0)
            return true;
    return false;
}

void Traffic::place(Car& car, bool initial) {
    car.id = next_id_++;
    for (int attempt = 0; attempt < 50; ++attempt) {
        car.lane = static_cast<int>(rng_() % 3) - 1;
        if (initial) {
            car.x = uniform(rng_, -40.0, 200.0);
            car.speed = uniform(rng_, 22.0, 38.0);
        } else if (rng_() % 2 == 0) {  // enter ahead, slower than the ego
            car.x = 240.0;
            car.speed = uniform(rng_, 20.0, ego_speed_ - 2.0);
        } else {  // enter behind, faster than the ego
            car.x = -50.0;
            car.speed = uniform(rng_, ego_speed_ + 2.0, 40.0);
        }
        car.target_lane = car.lane;
        if (!crowded(car)) break;
    }
    car.y = car.lane * kLaneWidth;
    car.vy = 0.0;
    car.target_lane = car.lane;
    car.desired_speed = car.speed;
    car.next_speed_change = t_ + uniform(rng_, 5.0, 20.0);
}

bool Traffic::lane_free(const Car& car, int lane) const {
    if (lane == 0 && std::abs(car.x) < 25.0) return false;  // the ego
    for (const Car& other : cars_)
        if (&other != &car && occupies(other.lane, other.target_lane, lane) && std::abs(other.x - car.x) < 25.0)
            return false;
    return true;
}

double Traffic::idm_accel(const Car& car) const {
    // Intelligent Driver Model (Treiber et al., 2000), following the nearest car
    // ahead in either lane this car occupies. The ego counts as a car too.
    double gap = 1e9, lead_speed = car.speed;
    auto consider = [&](double x, double speed, int lane, int target) {
        if (x <= car.x) return;
        if (!occupies(lane, target, car.lane) && !occupies(lane, target, car.target_lane)) return;
        const double g = x - car.x - kCarLength;
        if (g < gap) {
            gap = g;
            lead_speed = speed;
        }
    };
    consider(0.0, ego_speed_, 0, 0);
    for (const Car& other : cars_)
        if (&other != &car) consider(other.x, other.speed, other.lane, other.target_lane);

    const double a = 1.5, b = 2.0, headway = 1.2, min_gap = 2.0;
    const double free = 1.0 - std::pow(car.speed / car.desired_speed, 4);
    if (gap > 1e8) return a * free;
    const double dv = car.speed - lead_speed;
    const double wanted = min_gap + std::max(0.0, car.speed * headway + car.speed * dv / (2 * std::sqrt(a * b)));
    return a * (free - std::pow(wanted / std::max(gap, 0.5), 2));
}

void Traffic::step(double dt) {
    t_ += dt;
    for (Car& car : cars_) {
        if (t_ >= car.next_speed_change) {
            car.desired_speed = uniform(rng_, 20.0, 40.0);
            car.next_speed_change = t_ + uniform(rng_, 5.0, 20.0);
        }
        // Braking is capped at 8 m/s^2, a hard stop on a dry road.
        const double accel = std::clamp(idm_accel(car), -8.0, 2.0);
        car.speed = std::clamp(car.speed + accel * dt, 0.0, 42.0);
        car.x += (car.speed - ego_speed_) * dt;

        if (car.target_lane == car.lane && uniform(rng_, 0.0, 1.0) < dt / 25.0) {
            const int step = (car.lane == -1) ? 1 : (car.lane == 1) ? -1 : (rng_() % 2 ? 1 : -1);
            if (lane_free(car, car.lane + step)) {
                car.target_lane = car.lane + step;
                car.lane_start = t_;
                car.lane_duration = uniform(rng_, 3.0, 6.0);
            }
        }
        if (car.target_lane != car.lane) {
            // Smooth lateral motion: a cubic with zero lateral speed at both ends.
            const double u = std::min((t_ - car.lane_start) / car.lane_duration, 1.0);
            const double from = car.lane * kLaneWidth, to = car.target_lane * kLaneWidth;
            car.y = from + (to - from) * u * u * (3 - 2 * u);
            car.vy = (to - from) * 6 * u * (1 - u) / car.lane_duration;
            if (u >= 1.0) {
                car.lane = car.target_lane;
                car.vy = 0.0;
            }
        }
        if (car.x > 250.0 || car.x < -60.0) place(car, false);
    }
}

std::vector<Truth> Traffic::truths() const {
    std::vector<Truth> out;
    out.reserve(cars_.size());
    for (const Car& car : cars_) out.push_back({car.id, car.x, car.y, car.speed - ego_speed_, car.vy});
    return out;
}

}  // namespace fusion
