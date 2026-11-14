#include "fusion/ekf.hpp"

#include <algorithm>
#include <cmath>

namespace fusion {

namespace {

constexpr double kSpeedPriorLong = 10.0;  // m/s, relative speed along the road
constexpr double kSpeedPriorLat = 1.5;    // m/s, lateral speed, e.g. during a lane change

template <std::size_t M>
void finish(Innovation<M>& in, const Cov& P) {
    in.S = in.H * P * in.H.t() + in.R;
    in.S_inv = inverse(in.S);
    in.d2 = (in.y.t() * in.S_inv * in.y)(0, 0);
}

}  // namespace

double wrap_angle(double a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

void predict(State& x, Cov& P, double dt, const MotionNoise& q) {
    if (dt <= 0) return;
    Cov F = Cov::identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    // Discrete white-noise acceleration, separately along and across the road.
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    const double ql = q.accel_long * q.accel_long, qa = q.accel_lat * q.accel_lat;
    Cov Q;
    Q(0, 0) = dt4 / 4 * ql;
    Q(0, 2) = Q(2, 0) = dt3 / 2 * ql;
    Q(2, 2) = dt2 * ql;
    Q(1, 1) = dt4 / 4 * qa;
    Q(1, 3) = Q(3, 1) = dt3 / 2 * qa;
    Q(3, 3) = dt2 * qa;
    x = F * x;
    P = F * P * F.t() + Q;
}

Innovation<3> radar_innovation(const State& x, const Cov& P, const Detection& z, const SensorSpec& s) {
    const double px = x(0, 0), py = x(1, 0), vx = x(2, 0), vy = x(3, 0);
    const double r = std::max(std::hypot(px, py), 0.5);
    const double rate = (px * vx + py * vy) / r;
    Innovation<3> in;
    in.y = vec<3>({z.range - r, wrap_angle(z.bearing - std::atan2(py, px)), z.range_rate - rate});
    in.H(0, 0) = px / r;
    in.H(0, 1) = py / r;
    in.H(1, 0) = -py / (r * r);
    in.H(1, 1) = px / (r * r);
    in.H(2, 0) = (vx - rate * px / r) / r;
    in.H(2, 1) = (vy - rate * py / r) / r;
    in.H(2, 2) = px / r;
    in.H(2, 3) = py / r;
    const double sr = range_sigma(s, z.range);
    in.R(0, 0) = sr * sr;
    in.R(1, 1) = s.sigma_bearing * s.sigma_bearing;
    in.R(2, 2) = s.sigma_range_rate * s.sigma_range_rate;
    finish(in, P);
    return in;
}

Innovation<2> camera_innovation(const State& x, const Cov& P, const Detection& z, const SensorSpec& s) {
    const double px = x(0, 0), py = x(1, 0);
    const double r = std::max(std::hypot(px, py), 0.5);
    Innovation<2> in;
    in.y = vec<2>({z.range - r, wrap_angle(z.bearing - std::atan2(py, px))});
    in.H(0, 0) = px / r;
    in.H(0, 1) = py / r;
    in.H(1, 0) = -py / (r * r);
    in.H(1, 1) = px / (r * r);
    // The camera's range noise depends on range; the measured range is all the
    // tracker has, so it is used here.
    const double sr = range_sigma(s, z.range);
    in.R(0, 0) = sr * sr;
    in.R(1, 1) = s.sigma_bearing * s.sigma_bearing;
    finish(in, P);
    return in;
}

void init_from_detection(const Detection& z, const SensorSpec& s, State& x, Cov& P) {
    const double c = std::cos(z.bearing), sn = std::sin(z.bearing), r = z.range;
    const double sr = range_sigma(s, r);
    x = vec<4>({r * c, r * sn, 0.0, 0.0});
    P = Cov();
    // Position covariance: polar noise rotated into x, y.
    Mat<2, 2> J;
    J(0, 0) = c;
    J(0, 1) = -r * sn;
    J(1, 0) = sn;
    J(1, 1) = r * c;
    Mat<2, 2> polar;
    polar(0, 0) = sr * sr;
    polar(1, 1) = s.sigma_bearing * s.sigma_bearing;
    const Mat<2, 2> pos = J * polar * J.t();
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j) P(i, j) = pos(i, j);
    // Velocity prior from how highway traffic moves relative to the ego: large
    // speed differences along the road, small ones across it.
    P(2, 2) = kSpeedPriorLong * kSpeedPriorLong;
    P(3, 3) = kSpeedPriorLat * kSpeedPriorLat;

    if (s.kind == SensorKind::Radar) {
        // Fold in the range rate as a scalar Kalman update. With zero prior
        // velocity the predicted range rate is 0 and only the velocity terms of
        // the Jacobian are non-zero.
        Mat<1, 4> H;
        H(0, 2) = c;
        H(0, 3) = sn;
        Mat<1, 1> R;
        R(0, 0) = s.sigma_range_rate * s.sigma_range_rate;
        Innovation<1> in;
        in.y = vec<1>({z.range_rate});
        in.H = H;
        in.R = R;
        finish(in, P);
        update(x, P, in);
    }
}

}  // namespace fusion
