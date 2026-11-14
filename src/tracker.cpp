#include "fusion/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "fusion/assignment.hpp"

namespace fusion {

namespace {

double gated_cost(const Track& tr, const Detection& z, const SensorSpec& s) {
    const double d2 = s.kind == SensorKind::Radar ? radar_innovation(tr.x, tr.P, z, s).d2
                                                  : camera_innovation(tr.x, tr.P, z, s).d2;
    const double gate = s.kind == SensorKind::Radar ? kGate3 : kGate2;
    return d2 < gate ? d2 : std::numeric_limits<double>::infinity();
}

void apply(Track& tr, const Detection& z, const SensorSpec& s) {
    if (s.kind == SensorKind::Radar)
        update(tr.x, tr.P, radar_innovation(tr.x, tr.P, z, s));
    else
        update(tr.x, tr.P, camera_innovation(tr.x, tr.P, z, s));
}

}  // namespace

Tracker::Tracker(TrackerConfig cfg) : cfg_(cfg) {}

void Tracker::record(Track& track, bool hit) const {
    track.history.push_back(hit);
    if (static_cast<int>(track.history.size()) > cfg_.confirm_window)
        track.history.erase(track.history.begin());
    const auto hits = std::count(track.history.begin(), track.history.end(), true);
    if (hits >= cfg_.confirm_hits) track.confirmed = true;
}

bool Tracker::expired(const Track& track, double t) const {
    if (std::sqrt(track.P(0, 0) + track.P(1, 1)) > cfg_.max_position_sigma) return true;
    if (t - track.last_update > cfg_.coast_time) return true;
    // A tentative track that had its chances and was not confirmed is dropped.
    return !track.confirmed && static_cast<int>(track.history.size()) >= cfg_.confirm_window;
}

void Tracker::process(const std::vector<Detection>& scan, const SensorSpec& sensor, double t) {
    for (Track& tr : tracks_) {
        predict(tr.x, tr.P, t - tr.t, cfg_.motion);
        tr.t = t;
    }

    std::vector<std::vector<double>> cost(tracks_.size(), std::vector<double>(scan.size()));
    for (std::size_t i = 0; i < tracks_.size(); ++i)
        for (std::size_t j = 0; j < scan.size(); ++j) cost[i][j] = gated_cost(tracks_[i], scan[j], sensor);
    const double miss = sensor.kind == SensorKind::Radar ? kGate3 : kGate2;
    const std::vector<int> match = gnn_assign(cost, miss);

    std::vector<bool> used(scan.size(), false);
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        Track& tr = tracks_[i];
        const bool counts = !cfg_.confirm_by_source || tr.source == sensor.kind;
        if (match[i] >= 0) {
            apply(tr, scan[match[i]], sensor);
            used[match[i]] = true;
            tr.last_update = t;
            if (counts) record(tr, true);
        } else if (counts && in_view(sensor, tr.x(0, 0), tr.x(1, 0))) {
            record(tr, false);  // only scans that could have seen the track count as misses
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& tr) { return expired(tr, t); }),
                  tracks_.end());
    if (cfg_.prune_duplicates) prune_duplicates();

    for (std::size_t j = 0; j < scan.size(); ++j) {
        if (used[j]) continue;
        Track tr{next_id_++, {}, {}, t, t, false, {true}, sensor.kind};
        init_from_detection(scan[j], sensor, tr.x, tr.P);
        tracks_.push_back(tr);
    }
}

void Tracker::prune_duplicates() {
    // Tracks are kept in creation order, so for any pair the earlier one is older.
    std::vector<bool> drop(tracks_.size(), false);
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        if (drop[i]) continue;
        for (std::size_t j = i + 1; j < tracks_.size(); ++j) {
            if (drop[j]) continue;
            const Track &a = tracks_[i], &b = tracks_[j];
            Vec<2> d = vec<2>({a.x(0, 0) - b.x(0, 0), a.x(1, 0) - b.x(1, 0)});
            Mat<2, 2> S;
            for (std::size_t r = 0; r < 2; ++r)
                for (std::size_t k = 0; k < 2; ++k) S(r, k) = a.P(r, k) + b.P(r, k);
            if ((d.t() * inverse(S) * d)(0, 0) >= kDuplicateGate) continue;
            // Keep a confirmed track over a tentative one, otherwise the older.
            const bool keep_b = b.confirmed && !a.confirmed;
            drop[keep_b ? i : j] = true;
            if (keep_b) break;
        }
    }
    std::size_t k = 0;
    for (std::size_t i = 0; i < tracks_.size(); ++i)
        if (!drop[i]) tracks_[k++] = tracks_[i];
    tracks_.resize(k);
}

std::vector<Track> Tracker::confirmed_at(double t) const {
    std::vector<Track> out;
    for (const Track& tr : tracks_) {
        if (!tr.confirmed) continue;
        Track copy = tr;
        predict(copy.x, copy.P, t - copy.t, cfg_.motion);
        copy.t = t;
        out.push_back(copy);
    }
    return out;
}

}  // namespace fusion
