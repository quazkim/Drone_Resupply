#ifndef PDP_UTILS_H
#define PDP_UTILS_H

#include "pdp_types.h"
#include <vector>
#include <string>

// ============================================================
// === UTILITY FUNCTIONS (pdp_utils.h) ========================
// ============================================================

/// Khoảng cách Euclidean giữa 2 điểm
double euclideanDistance(double x1, double y1, double x2, double y2);

/**
 * @brief Kiểm tra các ràng buộc PDP của một lời giải.
 *
 * Kiểm tra:
 *  1. Precedence: P phải đứng trước DL tương ứng trên cùng tuyến xe.
 *  2. Resupply ordering: Mỗi gói hàng j trong resupply_vector tại node i
 *     phải có node i đứng trước hoặc chính là điểm giao của j trong lộ trình.
 *
 * @param solution Lời giải cần kiểm tra
 * @param data     Dữ liệu bài toán PDP
 * @return true nếu tất cả ràng buộc được thỏa mãn
 */
bool validatePDPConstraints(const PDPSolution& solution, const PDPData& data);

/**
 * @brief In chi tiết lời giải ra stdout.
 * Cuối hàm sẽ gọi printChromosome(solution.original_sequence) để hiển thị
 * chuỗi gen đã tạo ra lời giải này.
 */
void printSolution(const PDPSolution& solution, const PDPData& data);

/**
 * @brief Tính tổng chi phí lộ trình dựa trên ma trận khoảng cách.
 * Sử dụng trực tiếp node index (0-based) từ solution.routes.
 */
double calculateSolutionCost(const PDPSolution& solution,
                             const std::vector<std::vector<double>>& distMatrix);

/**
 * @brief In chromosome (vector<Gene>) ra stdout theo định dạng dễ debug.
 *
 * Định dạng:
 *   [Node: 1 | Resupply: {}] -> [Node: 2 | Resupply: {4, 5}]
 *   -> [Node: -1 (DEPOT)] -> [Node: 0 (SEPARATOR)] -> ...
 *
 * @param seq Chromosome cần in
 */
void printChromosome(const std::vector<Gene>& seq);

#endif // PDP_UTILS_H