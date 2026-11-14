#pragma once
#include <vector>

namespace fusion {

// Minimum-cost perfect matching on a square cost matrix (Hungarian algorithm,
// O(n^3)). Returns the column chosen for each row.
std::vector<int> hungarian(const std::vector<std::vector<double>>& cost);

// Global nearest neighbour assignment of rows (tracks) to columns (detections).
// Infinite costs mark pairs outside the gate. Leaving a row unassigned costs
// miss_cost, so a pair is only matched if its cost is below miss_cost.
// Returns the column for each row, or -1.
std::vector<int> gnn_assign(const std::vector<std::vector<double>>& cost, double miss_cost);

}  // namespace fusion
