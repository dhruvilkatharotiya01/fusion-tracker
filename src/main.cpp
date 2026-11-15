// fusion_eval: compare radar-only, camera-only and fused tracking over many
// simulated drives, and optionally write per-run results and a trace.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "fusion/experiment.hpp"

using namespace fusion;

namespace {

void usage() {
    std::puts("usage: fusion_eval [--runs N] [--duration S] [--seed N] [--csv FILE] [--trace FILE]\n"
              "                   [--q-long A] [--q-lat A] [--no-prune]\n"
              "                   [--any-sensor-confirms] [--confirm HITS WINDOW]\n"
              "                   [--no-occlusion] [--no-occlusion-coasting]\n"
              "  --trace writes truth and fused tracks for the first run, for tools/plot_trace.py");
}

}  // namespace

int main(int argc, char** argv) {
    int runs = 30;
    double duration = 60.0;
    unsigned long long seed = 1;
    std::string csv_path, trace_path;
    TrackerConfig cfg;
    bool occlusion = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has_value = i + 1 < argc;
        if (arg == "--runs" && has_value) runs = std::atoi(argv[++i]);
        else if (arg == "--duration" && has_value) duration = std::atof(argv[++i]);
        else if (arg == "--seed" && has_value) seed = std::strtoull(argv[++i], nullptr, 10);
        else if (arg == "--csv" && has_value) csv_path = argv[++i];
        else if (arg == "--trace" && has_value) trace_path = argv[++i];
        else if (arg == "--q-long" && has_value) cfg.motion.accel_long = std::atof(argv[++i]);
        else if (arg == "--q-lat" && has_value) cfg.motion.accel_lat = std::atof(argv[++i]);
        else if (arg == "--no-prune") cfg.prune_duplicates = false;
        else if (arg == "--any-sensor-confirms") cfg.confirm_by_source = false;
        else if (arg == "--no-occlusion") occlusion = false;
        else if (arg == "--no-occlusion-coasting") cfg.occlusion_coasting = false;
        else if (arg == "--occlusion-coasting") cfg.occlusion_coasting = true;
        else if (arg == "--confirm" && i + 2 < argc) {
            cfg.confirm_hits = std::atoi(argv[++i]);
            cfg.confirm_window = std::atoi(argv[++i]);
        }
        else {
            usage();
            return arg == "--help" ? 0 : 2;
        }
    }

    std::FILE* csv = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
    if (csv)
        std::fputs("mode,run,rmse_lon,rmse_lat,rmse_vel,mean_nees,lane_accuracy,coverage,"
                   "false_tracks,id_switches_per_min\n", csv);

    const Mode modes[] = {Mode::Radar, Mode::Camera, Mode::Fused};
    std::vector<std::vector<Summary>> per_run(3);
    std::vector<Summary> overall;
    std::printf("%d runs x %.0f s, seed %llu, q_long %.2f, q_lat %.2f\n\n", runs, duration, seed,
                cfg.motion.accel_long, cfg.motion.accel_lat);
    std::printf("%-7s %8s %8s %8s %6s %6s %6s %7s %7s %7s   lateral RMSE by range\n", "mode", "lon m",
                "lat m", "vel m/s", "NEES", "lane", "cover", "false", "sw/min", "ttt s");

    for (int k = 0; k < 3; ++k) {
        Totals all;
        for (int r = 0; r < runs; ++r) {
            RunOptions opt;
            opt.mode = modes[k];
            opt.seed = seed * 1000 + r;
            opt.duration = duration;
            opt.tracker = cfg;
            opt.occlusion = occlusion;
            std::FILE* trace = nullptr;
            if (!trace_path.empty() && r == 0 && modes[k] == Mode::Fused) {
                trace = std::fopen(trace_path.c_str(), "w");
                if (trace) std::fputs("t,kind,id,x,y\n", trace);
            }
            opt.trace = trace;
            const Totals t = run_once(opt);
            if (trace) std::fclose(trace);
            all.add(t);
            const Summary s = summarize(t);
            per_run[k].push_back(s);
            if (csv)
                std::fprintf(csv, "%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", mode_name(modes[k]), r,
                             s.rmse_lon, s.rmse_lat, s.rmse_vel, s.mean_nees, s.lane_accuracy, s.coverage,
                             s.false_tracks, s.id_switches_per_min);
        }
        const Summary s = summarize(all);
        overall.push_back(s);
        std::printf("%-7s %8.2f %8.2f %8.2f %6.1f %5.1f%% %5.1f%% %7.2f %7.2f %7.2f   %.2f / %.2f / %.2f\n",
                    mode_name(modes[k]), s.rmse_lon, s.rmse_lat, s.rmse_vel, s.mean_nees,
                    100 * s.lane_accuracy, 100 * s.coverage, s.false_tracks, s.id_switches_per_min,
                    s.time_to_track,
                    s.rmse_lat_by_range[0], s.rmse_lat_by_range[1], s.rmse_lat_by_range[2]);
    }
    if (csv) std::fclose(csv);

    if (occlusion) {
        std::printf("\noccluded cars: %.1f%% of scored car-steps are less than half visible\n",
                    100 * overall[0].hidden_share);
        std::printf("%-7s %10s %10s %10s\n", "", "lat vis m", "lat hid m", "cover hid");
        for (int k = 0; k < 3; ++k)
            std::printf("hidden  %-7s %8.2f %10.2f %9.1f%%\n", mode_name(modes[k]), overall[k].rmse_lat_visible,
                        overall[k].rmse_lat_hidden, 100 * overall[k].coverage_hidden);
    }

    // Paired comparison: the same drives and detections in every mode.
    int better_lat = 0, better_lon = 0;
    for (int r = 0; r < runs; ++r) {
        if (per_run[2][r].rmse_lat < per_run[0][r].rmse_lat) ++better_lat;
        if (per_run[2][r].rmse_lon < per_run[1][r].rmse_lon) ++better_lon;
    }
    std::printf("\nfused lateral error below radar-only in %d of %d runs\n", better_lat, runs);
    std::printf("fused longitudinal error below camera-only in %d of %d runs\n", better_lon, runs);
    std::puts("lateral RMSE ranges: 5-30 m / 30-55 m / 55-80 m ahead. NEES should be near 4 if the filter is consistent.");
    std::puts("ttt: time to track, mean seconds a car spends in the scored region before it has a confirmed track.");
    return 0;
}
