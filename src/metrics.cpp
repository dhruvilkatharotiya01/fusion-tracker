#include "fusion/metrics.hpp"

#include <cmath>
#include <limits>

#include "fusion/assignment.hpp"
#include "fusion/traffic.hpp"

namespace fusion {

bool in_region(double x, double y) {
    return x >= kRangeBins.front() && x <= kRangeBins.back() && std::abs(std::atan2(y, x)) < 28 * kPi / 180;
}

int lane_of(double y) { return static_cast<int>(std::lround(y / kLaneWidth)); }

void Totals::add(const Totals& o) {
    sq_lon += o.sq_lon;
    sq_lat += o.sq_lat;
    sq_vel += o.sq_vel;
    nees += o.nees;
    matched += o.matched;
    truths += o.truths;
    lane_correct += o.lane_correct;
    false_tracks += o.false_tracks;
    id_switches += o.id_switches;
    steps += o.steps;
    seconds += o.seconds;
    delay_sum += o.delay_sum;
    delay_n += o.delay_n;
    for (std::size_t b = 0; b < 3; ++b) {
        sq_lat_bin[b] += o.sq_lat_bin[b];
        n_bin[b] += o.n_bin[b];
    }
}

Summary summarize(const Totals& t) {
    const double m = t.matched ? static_cast<double>(t.matched) : std::numeric_limits<double>::quiet_NaN();
    Summary s{};
    s.rmse_lon = std::sqrt(t.sq_lon / m);
    s.rmse_lat = std::sqrt(t.sq_lat / m);
    s.rmse_vel = std::sqrt(t.sq_vel / m);
    s.mean_nees = t.nees / m;
    s.lane_accuracy = t.lane_correct / m;
    s.coverage = t.truths ? static_cast<double>(t.matched) / t.truths : 0.0;
    s.false_tracks = t.steps ? static_cast<double>(t.false_tracks) / t.steps : 0.0;
    s.id_switches_per_min = t.seconds > 0 ? t.id_switches / (t.seconds / 60.0) : 0.0;
    s.time_to_track = t.delay_n ? t.delay_sum / t.delay_n : 0.0;
    for (std::size_t b = 0; b < 3; ++b)
        s.rmse_lat_by_range[b] = t.n_bin[b] ? std::sqrt(t.sq_lat_bin[b] / t.n_bin[b])
                                            : std::numeric_limits<double>::quiet_NaN();
    return s;
}

void Evaluator::add(const std::vector<Truth>& truths, const std::vector<Track>& tracks, double dt) {
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> cost(truths.size(), std::vector<double>(tracks.size(), inf));
    for (std::size_t i = 0; i < truths.size(); ++i)
        for (std::size_t j = 0; j < tracks.size(); ++j) {
            const double dx = (tracks[j].x(0, 0) - truths[i].x) / kMatchLon;
            const double dy = (tracks[j].x(1, 0) - truths[i].y) / kMatchLat;
            const double d = std::hypot(dx, dy);
            if (d < 1.0) cost[i][j] = d;
        }
    const std::vector<int> match = gnn_assign(cost, 1.0);

    std::vector<bool> track_used(tracks.size(), false);
    for (std::size_t i = 0; i < truths.size(); ++i) {
        if (match[i] >= 0) track_used[match[i]] = true;
        const Truth& tr = truths[i];
        if (!in_region(tr.x, tr.y)) continue;
        ++totals_.truths;
        if (!seen_[tr.id] && !waiting_.count(tr.id)) waiting_[tr.id] = now_;
        if (match[i] < 0) continue;
        if (auto w = waiting_.find(tr.id); w != waiting_.end()) {
            // Cars already in the region when scoring starts are skipped.
            if (now_ > 0) {
                totals_.delay_sum += now_ - w->second;
                ++totals_.delay_n;
            }
            waiting_.erase(w);
            seen_[tr.id] = true;
        }
        const Track& tk = tracks[match[i]];
        ++totals_.matched;

        const State e = tk.x - vec<4>({tr.x, tr.y, tr.vx, tr.vy});
        totals_.sq_lon += e(0, 0) * e(0, 0);
        totals_.sq_lat += e(1, 0) * e(1, 0);
        totals_.sq_vel += e(2, 0) * e(2, 0) + e(3, 0) * e(3, 0);
        totals_.nees += (e.t() * inverse(tk.P) * e)(0, 0);
        if (lane_of(tk.x(1, 0)) == lane_of(tr.y)) ++totals_.lane_correct;
        for (std::size_t b = 0; b < 3; ++b)
            if (tr.x >= kRangeBins[b] && tr.x < kRangeBins[b + 1]) {
                totals_.sq_lat_bin[b] += e(1, 0) * e(1, 0);
                ++totals_.n_bin[b];
            }

        auto it = last_track_.find(tr.id);
        if (it != last_track_.end() && it->second != tk.id) ++totals_.id_switches;
        last_track_[tr.id] = tk.id;
    }
    for (std::size_t j = 0; j < tracks.size(); ++j)
        if (!track_used[j] && in_region(tracks[j].x(0, 0), tracks[j].x(1, 0))) ++totals_.false_tracks;
    ++totals_.steps;
    totals_.seconds += dt;
    now_ += dt;
}

}  // namespace fusion
