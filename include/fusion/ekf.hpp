#pragma once
// Extended Kalman filter with a constant-velocity model and polar measurement
// models for radar (range, bearing, range rate) and camera (range, bearing).
#include "fusion/matrix.hpp"
#include "fusion/sensors.hpp"

namespace fusion {

using State = Vec<4>;  // x, y, vx, vy relative to the ego
using Cov = Mat<4, 4>;

struct MotionNoise {
    double accel_long = 2.5;  // std of unmodelled acceleration, m/s^2 (chosen so NEES is near 4)
    double accel_lat = 1.0;
};

double wrap_angle(double a);
void predict(State& x, Cov& P, double dt, const MotionNoise& q);

template <std::size_t M>
struct Innovation {
    Vec<M> y;       // measurement minus prediction
    Mat<M, 4> H;    // measurement Jacobian
    Mat<M, M> R;    // measurement noise
    Mat<M, M> S;    // innovation covariance
    Mat<M, M> S_inv;
    double d2 = 0;  // squared Mahalanobis distance, used for gating
};

Innovation<3> radar_innovation(const State& x, const Cov& P, const Detection& z, const SensorSpec& s);
Innovation<2> camera_innovation(const State& x, const Cov& P, const Detection& z, const SensorSpec& s);

template <std::size_t M>
void update(State& x, Cov& P, const Innovation<M>& in) {
    const Mat<4, M> K = P * in.H.t() * in.S_inv;
    x = x + K * in.y;
    // Joseph form keeps P symmetric and positive definite despite rounding.
    const Cov I_KH = Cov::identity() - K * in.H;
    P = I_KH * P * I_KH.t() + K * in.R * K.t();
}

// Start a track from one detection, with a velocity prior that allows large
// relative speeds along the road but only small ones across it. A radar range
// rate is then applied as a measurement of the radial velocity.
void init_from_detection(const Detection& z, const SensorSpec& s, State& x, Cov& P);

}  // namespace fusion
