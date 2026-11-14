#include "fusion/assignment.hpp"

#include <cmath>
#include <limits>

namespace fusion {

std::vector<int> hungarian(const std::vector<std::vector<double>>& cost) {
    // Potentials-based version; rows and columns are 1-indexed internally.
    const int n = static_cast<int>(cost.size());
    const double inf = std::numeric_limits<double>::max();
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<int> owner(n + 1, 0), way(n + 1, 0);
    for (int i = 1; i <= n; ++i) {
        owner[0] = i;
        int j0 = 0;
        std::vector<double> min_v(n + 1, inf);
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const int i0 = owner[j0];
            double delta = inf;
            int j1 = 0;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < min_v[j]) {
                    min_v[j] = cur;
                    way[j] = j0;
                }
                if (min_v[j] < delta) {
                    delta = min_v[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[owner[j]] += delta;
                    v[j] -= delta;
                } else {
                    min_v[j] -= delta;
                }
            }
            j0 = j1;
        } while (owner[j0] != 0);
        do {
            const int j1 = way[j0];
            owner[j0] = owner[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    std::vector<int> col_of_row(n, -1);
    for (int j = 1; j <= n; ++j)
        if (owner[j] > 0) col_of_row[owner[j] - 1] = j - 1;
    return col_of_row;
}

std::vector<int> gnn_assign(const std::vector<std::vector<double>>& cost, double miss_cost) {
    const int n = static_cast<int>(cost.size());
    const int m = n ? static_cast<int>(cost[0].size()) : 0;
    std::vector<int> result(n, -1);
    if (n == 0 || m == 0) return result;

    // Square matrix of size n + m with dummy columns ("track missed") and dummy
    // rows ("detection unused"), so leaving things unassigned is always possible.
    const double big = 1e9;
    const int size = n + m;
    std::vector<std::vector<double>> c(size, std::vector<double>(size, big));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j)
            if (std::isfinite(cost[i][j])) c[i][j] = cost[i][j];
        c[i][m + i] = miss_cost;
    }
    for (int j = 0; j < m; ++j) c[n + j][j] = 0.0;
    for (int i = n; i < size; ++i)
        for (int j = m; j < size; ++j) c[i][j] = 0.0;

    const std::vector<int> match = hungarian(c);
    for (int i = 0; i < n; ++i)
        if (match[i] < m && std::isfinite(cost[i][match[i]])) result[i] = match[i];
    return result;
}

}  // namespace fusion
