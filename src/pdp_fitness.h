#ifndef PDP_FITNESS_H
#define PDP_FITNESS_H

#include "pdp_types.h"
#include "pdp_cache.h"
#include <vector>
#include <string>



// ============================================================
// === API MỚI: GIẢI MÃ CHUỖI SeqStop ===
// ============================================================

/**
 * @brief Giải mã chuỗi SeqStop thành lộ trình chi tiết cho 2 xe tải + drone.
 *
 * Quy ước phân chia lộ trình:
 *   - Phần tử với node_id == 0 là vách ngăn: trước thuộc Truck 1, sau thuộc Truck 2.
 *   - Phần tử với node_id == -1 là lệnh quay về Depot: dỡ C2, nhận C1.
 *   - Phần tử với node_id > 0 là điểm khách hàng.
 *
 * Logic tại mỗi điểm dừng (node_id > 0):
 *   1. Delivery (nếu hàng đã có trên xe) → giảm tải
 *   2. Drone resupply (nhận hàng từ drone theo resupply_vector) → tăng tải
 *   3. Pickup (lấy hàng lên xe nếu điểm là loại P) → tăng tải
 *   4. Kiểm tra tải trọng (current_load <= M_T) trước khi rời điểm
 *
 * @param seq       Chuỗi mã hóa theo định dạng mới (vector<SeqStop>)
 * @param data      Thông tin bài toán PDP
 * @param throw_on_infeasible  Nếu true: ném InfeasibleException khi vi phạm capacity.
 *                              Nếu false (mặc định): cộng penalty vào totalPenalty.
 * @return PDPSolution chứa makespan (totalCost), penalty, chi tiết xe tải, drone.
 */
PDPSolution decode_sequence(
    const std::vector<SeqStop>& seq,
    const PDPData& data,
    bool throw_on_infeasible = false
);

/**
 * @brief In lộ trình chi tiết của 2 xe tải và thứ tự bay của drone ra stdout.
 * Dùng để debug / visualization kết quả decode_sequence().
 *
 * @param sol  Kết quả từ decode_sequence()
 * @param data Thông tin bài toán
 */
void print_decoded_routes(const PDPSolution& sol, const PDPData& data);

/**
 * @brief Bridge: chuyển vector<int> node_ids → vector<SeqStop>, decode, trả PDPSolution.
 * Sử dụng SolutionCache để tránh decode lại cùng một sequence.
 *
 * @param sequence  Chuỗi node_id (bao gồm 0=separator, -1=depot-return, >0=customer)
 * @param data      Thông tin bài toán
 * @param cache     Cache object
 * @return PDPSolution
 */
PDPSolution evaluateWithCache(const std::vector<int>& sequence,
                              const PDPData& data,
                              SolutionCache& cache);

/**
 * @brief Bridge đơn giản (không cache): chuyển vector<int> → decode_sequence.
 *
 * @param sequence  Chuỗi node_id
 * @param data      Thông tin bài toán
 * @return PDPSolution
 */
PDPSolution decodeAndEvaluate(const std::vector<int>& sequence,
                              const PDPData& data);

#endif // PDP_FITNESS_H
