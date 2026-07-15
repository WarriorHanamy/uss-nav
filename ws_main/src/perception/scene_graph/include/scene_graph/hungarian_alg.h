#ifndef HUNGARIAN_MATCHER_H
#define HUNGARIAN_MATCHER_H

#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <algorithm>
#include <numeric>

#include <Eigen/Dense>

namespace Hungarian {

/**
 * Hungarian algorithm for minimum-cost assignment.
 *
 * Solves the assignment problem for matching old and new 3D mean
 * vectors (e.g., object centroids between frames). The `matchMeans`
 * static method is the public interface.
 */
class HungarianMatcher {
public:
    /**
     * Match old and new mean vectors minimizing total cost (sum of squared distances).
     *
     * @param[in] old_means  Map of old ID to mean vector
     * @param[in] new_means  Map of new ID to mean vector
     * @return Map from new ID to matched old ID
     */
    static std::map<int, int> matchMeans(
        const std::map<int, Eigen::Vector3d>& old_means,
        const std::map<int, Eigen::Vector3d>& new_means) {

        if (old_means.empty() || new_means.empty()) {
            return {};
        }

        // 1. Extract IDs and vectors into arrays for stable indexing
        std::vector<int> old_ids, new_ids;
        std::vector<Eigen::Vector3d> old_vecs, new_vecs;
        for(const auto& pair : old_means) {
            old_ids.push_back(pair.first);
            old_vecs.push_back(pair.second);
        }
        for(const auto& pair : new_means) {
            new_ids.push_back(pair.first);
            new_vecs.push_back(pair.second);
        }

        // 2. Determine which set is "rows" (smaller) and which is "cols" (larger)
        bool swapped = old_ids.size() > new_ids.size();

        const auto& row_ids = swapped ? new_ids : old_ids;
        const auto& row_vecs = swapped ? new_vecs : old_vecs;
        const auto& col_ids = swapped ? old_ids : new_ids;
        const auto& col_vecs = swapped ? old_vecs : new_vecs;

        size_t num_rows = row_ids.size();
        size_t num_cols = col_ids.size();
        size_t matrix_size = num_cols; // Make cost matrix square

        // 3. Build cost matrix
        std::vector<std::vector<double>> cost_matrix(matrix_size, std::vector<double>(matrix_size, 0));
        for (size_t i = 0; i < num_rows; ++i) {
            for (size_t j = 0; j < num_cols; ++j) {
                cost_matrix[i][j] = (row_vecs[i] - col_vecs[j]).squaredNorm();
            }
        }

        // Fill dummy rows with a high cost
        double high_cost = std::numeric_limits<double>::max() / 2.0;
        for (size_t i = num_rows; i < matrix_size; ++i) {
            std::fill(cost_matrix[i].begin(), cost_matrix[i].end(), high_cost);
        }

        // 4. Solve using Hungarian algorithm
        std::vector<int> assignment; // assignment[i] = j means row i matched to column j
        solve(cost_matrix, assignment);

        // 5. Parse results, map indices back to IDs
        std::map<int, int> new_to_old_id_map;
        double total_real_cost = 0;

        for (size_t i = 0; i < num_rows; ++i) { // Only iterate real rows
            int col_idx = assignment[i];
            if (col_idx < num_cols) { // Ensure match is to a real column
                int row_id = row_ids[i];
                int col_id = col_ids[col_idx];

                int old_id, new_id;
                if (swapped) { // rows are new, columns are old
                    new_id = row_id;
                    old_id = col_id;
                } else { // rows are old, columns are new
                    new_id = col_id;
                    old_id = row_id;
                }
                new_to_old_id_map[new_id] = old_id;
                total_real_cost += cost_matrix[i][col_idx];
            }
        }

        std::cout << "Match complete, total cost (sum of squared distances): " << total_real_cost << std::endl;
        return new_to_old_id_map;
    }

private:
    /**
     * Core Hungarian algorithm implementation for minimum-cost assignment.
     *
     * @param[in]  costMatrix  Square cost matrix
     * @param[out] assignment  Result vector, assignment[i] = j means row i maps to column j
     * @return Minimum total cost
     */
    static double solve(const std::vector<std::vector<double>>& costMatrix, std::vector<int>& assignment) {
        const int n = costMatrix.size();
        if (n == 0) return 0.0;

        assignment.assign(n, -1);
        std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
        std::vector<int> p(n + 1, 0), way(n + 1, 0);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n + 1, std::numeric_limits<double>::max());
            std::vector<bool> used(n + 1, false);
            do {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                double delta = std::numeric_limits<double>::max();
                for (int j = 1; j <= n; ++j) {
                    if (!used[j]) {
                        double cur = costMatrix[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                for (int j = 0; j <= n; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }

        std::vector<int> result(n);
        for (int j = 1; j <= n; ++j) {
            result[p[j] - 1] = j - 1;
        }
        assignment = result;

        return -v[0]; // Return minimum cost
    }
};

} // namespace Hungarian

/*
// =================================================================
//                      How to use the new Class
// =================================================================
int main() {
    // --- Example data: fewer old IDs than new IDs ---
    // Old means
    std::map<int, Eigen::Vector3d> old_data;
    old_data[20] = Eigen::Vector3d(5.0, 5.0, 5.0);    // ID 20
    old_data[30] = Eigen::Vector3d(10.0, 10.0, 10.0); // ID 30

    // New means
    std::map<int, Eigen::Vector3d> new_data;
    new_data[101] = Eigen::Vector3d(5.1, 5.2, 5.0);   // ID 101, closest to ID 20
    new_data[102] = Eigen::Vector3d(0.9, 1.1, 0.8);   // ID 102, should not match
    new_data[103] = Eigen::Vector3d(10.5, 9.8, 10.1); // ID 103, closest to ID 30

    // --- Run matching ---
    std::map<int, int> matches = Hungarian::HungarianMatcher::matchMeans(old_data, new_data);

    // --- Print results ---
    std::cout << "\nMatch results (new ID -> old ID):" << std::endl;
    for (const auto& match : matches) {
        std::cout << "New ID: " << match.first << "  ->  matched to old ID: " << match.second << std::endl;
    }

    return 0;
}
*/

#endif // HUNGARIAN_MATCHER_H