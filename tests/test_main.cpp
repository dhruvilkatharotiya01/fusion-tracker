// Unit and behaviour tests. A tiny runner keeps the project free of
// dependencies; each TEST registers itself and CHECK records failures.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include "fusion/assignment.hpp"
#include "fusion/ekf.hpp"
#include "fusion/experiment.hpp"
#include "fusion/tracker.hpp"
#include "fusion/traffic.hpp"

using namespace fusion;

namespace {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};
std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}
struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};
int failures = 0;

}  // namespace

#define TEST(name)                                   \
    static void name();                              \
    static Registrar registrar_##name(#name, name);  \
    static void name()

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            ++failures;                                                   \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::abs((a) - (b)) <= (tol))

TEST(inverse_times_matrix_is_identity) {
    Mat<3, 3> m;
    m.a = {4, 1, 2, 1, 3, 0, 2, 0, 5};
    const Mat<3, 3> p = inverse(m) * m;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c) CHECK_NEAR(p(r, c), r == c ? 1.0 : 0.0, 1e-12);
}

TEST(hungarian_matches_brute_force) {
    std::mt19937_64 rng(3);
    std::uniform_real_distribution<double> u(-5, 10);
    for (int trial = 0; trial < 200; ++trial) {
        const int n = 1 + trial % 6;
        std::vector<std::vector<double>> c(n, std::vector<double>(n));
        for (auto& row : c)
            for (auto& v : row) v = u(rng);
        std::vector<int> perm(n);
        std::iota(perm.begin(), perm.end(), 0);
        double best = 1e18;
        do {
            double s = 0;
            for (int i = 0; i < n; ++i) s += c[i][perm[i]];
            best = std::min(best, s);
        } while (std::next_permutation(perm.begin(), perm.end()));
        const auto match = hungarian(c);
        double got = 0;
        for (int i = 0; i < n; ++i) got += c[i][match[i]];
        CHECK_NEAR(got, best, 1e-9);
    }
}

TEST(gnn_leaves_gated_out_pairs_unassigned) {
    const double inf = std::numeric_limits<double>::infinity();
    // Track 0 can only take detection 1; track 1 is outside every gate.
    const std::vector<std::vector<double>> c = {{inf, 2.0, inf}, {inf, inf, inf}};
    const auto m = gnn_assign(c, 10.0);
    CHECK(m[0] == 1);
    CHECK(m[1] == -1);
}

TEST(gnn_prefers_the_globally_cheapest_assignment) {
    // Greedy would give detection 0 to track 0 (cost 1) and leave track 1 with 9.
    const std::vector<std::vector<double>> c = {{1.0, 2.0}, {2.0, 9.0}};
    const auto m = gnn_assign(c, 20.0);
    CHECK(m[0] == 1);
    CHECK(m[1] == 0);
}

TEST(radar_jacobian_matches_finite_differences) {
    const SensorSpec s = radar_spec();
    const State x = vec<4>({40.0, 3.0, -5.0, 0.8});
    const Cov P = Cov::identity();
    const Detection z{SensorKind::Radar, 0, 40.0, 0.07, -4.9, 0};
    const auto in = radar_innovation(x, P, z, s);
    const double h = 1e-6;
    for (std::size_t k = 0; k < 4; ++k) {
        State xp = x;
        xp(k, 0) += h;
        const auto ip = radar_innovation(xp, P, z, s);
        for (std::size_t row = 0; row < 3; ++row) {
            // The innovation is z - h(x), so its derivative is -H.
            const double numeric = -(ip.y(row, 0) - in.y(row, 0)) / h;
            CHECK_NEAR(numeric, in.H(row, k), 1e-5);
        }
    }
}

TEST(wrap_angle_stays_in_range) {
    CHECK_NEAR(wrap_angle(3 * kPi), kPi, 1e-12);
    CHECK_NEAR(wrap_angle(-3.5 * kPi), 0.5 * kPi, 1e-12);
    CHECK_NEAR(wrap_angle(0.25), 0.25, 1e-15);
}

TEST(filter_is_consistent_on_a_constant_velocity_target) {
    // Truth moves at constant velocity, so the only errors are sensor noise.
    // The average NEES over many runs should then be close to 4.
    const SensorSpec s = radar_spec();
    const MotionNoise q{1e-3, 1e-3};  // truth has no acceleration
    std::mt19937_64 rng(11);
    std::normal_distribution<double> g(0, 1);
    double nees = 0;
    const int runs = 300;
    for (int run = 0; run < runs; ++run) {
        double tx = 60, ty = 2;
        const double tvx = -4, tvy = 0.3;
        State x;
        Cov P;
        for (int k = 0; k <= 100; ++k) {
            tx += tvx * 0.05;
            ty += tvy * 0.05;
            const double r = std::hypot(tx, ty);
            const Detection z{SensorKind::Radar, k * 0.05, r + s.sigma_range_const * g(rng),
                              std::atan2(ty, tx) + s.sigma_bearing * g(rng),
                              (tx * tvx + ty * tvy) / r + s.sigma_range_rate * g(rng), 0};
            if (k == 0) {
                init_from_detection(z, s, x, P);
                continue;
            }
            predict(x, P, 0.05, q);
            update(x, P, radar_innovation(x, P, z, s));
        }
        const State e = x - vec<4>({tx, ty, tvx, tvy});
        nees += (e.t() * inverse(P) * e)(0, 0);
    }
    nees /= runs;
    std::printf("  mean NEES %.2f (expect 4)\n", nees);
    CHECK(nees > 3.3 && nees < 4.8);
}

TEST(sensor_only_reports_cars_in_its_field_of_view) {
    SensorSpec quiet = camera_spec();
    quiet.clutter_mean = 0;
    quiet.p_detect = 1;
    std::mt19937_64 rng(5);
    const std::vector<Truth> cars = {{1, 50, 0, 0, 0}, {2, 50, 40, 0, 0}, {3, 120, 0, 0, 0}, {4, -20, 0, 0, 0}};
    const auto dets = sense(quiet, cars, 0, rng);
    CHECK(dets.size() == 1);
    CHECK(dets.size() == 1 && dets[0].truth_id == 1);
}

TEST(traffic_cars_keep_their_distance_where_sensors_look) {
    // Rarely, a car entering 50 m behind the ego finds no free slot and overlaps
    // another car. That is outside every sensor's view, so only 0-160 m ahead is checked.
    double closest = 1e9;
    for (std::uint64_t seed = 0; seed < 5; ++seed) {
        Traffic traffic(seed);
        for (int k = 0; k < 600 * 60; ++k) {
            traffic.step(1.0 / 600);
            const auto cars = traffic.truths();
            for (std::size_t i = 0; i < cars.size(); ++i)
                for (std::size_t j = i + 1; j < cars.size(); ++j)
                    if (cars[i].x > 0 && cars[i].x < 160 && std::abs(cars[i].y - cars[j].y) < 1.8)
                        closest = std::min(closest, std::abs(cars[i].x - cars[j].x));
        }
    }
    std::printf("  closest same-lane spacing %.1f m\n", closest);
    CHECK(closest > 4.5);  // no car drives through another
}

TEST(single_car_gives_one_stable_track) {
    SensorSpec radar = radar_spec();
    radar.clutter_mean = 0;
    Tracker tracker;
    std::mt19937_64 rng(2);
    Truth car{7, 50, 0, -3, 0};
    for (int k = 1; k <= 200; ++k) {
        car.x += car.vx * 0.05;
        tracker.process(sense(radar, {car}, k * 0.05, rng), radar, k * 0.05);
    }
    const auto tracks = tracker.confirmed_at(10.0);
    CHECK(tracks.size() == 1);
    CHECK(tracks.size() == 1 && tracks[0].id == 1);
    CHECK(tracks.size() == 1 && std::abs(tracks[0].x(0, 0) - car.x) < 1.0);
}

TEST(duplicate_tracks_are_pruned) {
    SensorSpec radar = radar_spec();
    radar.clutter_mean = 0;
    radar.p_detect = 1;
    Tracker tracker;
    std::mt19937_64 rng(4);
    const Truth car{1, 40, 0, 0, 0};
    // Two detections of the same car in one scan start two tracks.
    auto scan = sense(radar, {car}, 0.05, rng);
    scan.push_back(scan[0]);
    scan[1].range += 0.2;
    tracker.process(scan, radar, 0.05);
    CHECK(tracker.tracks().size() == 2);
    tracker.process(sense(radar, {car}, 0.10, rng), radar, 0.10);
    CHECK(tracker.tracks().size() == 1);
}

TEST(runs_are_reproducible) {
    RunOptions a;
    a.mode = Mode::Fused;
    a.duration = 15;
    a.seed = 9;
    const Totals ta = run_once(a), tb = run_once(a);
    CHECK(ta.matched == tb.matched && ta.sq_lat == tb.sq_lat && ta.id_switches == tb.id_switches);
}

TEST(fusion_beats_each_sensor_where_it_is_weak) {
    RunOptions opt;
    opt.duration = 40;
    Totals radar, camera, fused;
    for (int seed = 0; seed < 4; ++seed) {
        opt.seed = 500 + seed;
        opt.mode = Mode::Radar;
        radar.add(run_once(opt));
        opt.mode = Mode::Camera;
        camera.add(run_once(opt));
        opt.mode = Mode::Fused;
        fused.add(run_once(opt));
    }
    const Summary r = summarize(radar), c = summarize(camera), f = summarize(fused);
    std::printf("  lateral m: radar %.2f fused %.2f | longitudinal m: camera %.2f fused %.2f\n", r.rmse_lat,
                f.rmse_lat, c.rmse_lon, f.rmse_lon);
    CHECK(f.rmse_lat < r.rmse_lat);
    CHECK(f.rmse_lon < c.rmse_lon);
}

int main() {
    for (const auto& t : registry()) {
        const int before = failures;
        std::printf("%s\n", t.name);
        t.fn();
        if (failures > before) std::printf("  -> failed\n");
    }
    std::printf("\n%zu tests, %d failed checks\n", registry().size(), failures);
    return failures ? 1 : 0;
}
