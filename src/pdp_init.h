#ifndef PDP_INIT_H
#define PDP_INIT_H

#include "pdp_types.h"
#include <vector>
#include <random>

using namespace std;

// === DISTANCE ACCESSOR FUNCTIONS ===

/**
 * @brief Retrieve truck distance between two nodes from Manhattan matrix.
 * @param data PDP instance with distance matrices
 * @param nodeA_id, nodeB_id 0-based node identifiers
 * @return Distance value from truck distance matrix
 */
double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id);

/**
 * @brief Retrieve drone distance between two nodes from Euclidean matrix.
 * @param data PDP instance with distance matrices
 * @param nodeA_id, nodeB_id 0-based node identifiers
 * @return Distance value from drone distance matrix
 */
double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id);

// === CHROMOSOME BUILDING UTILITIES ===

/**
 * @brief Chèn số 0 (Separator) cân bằng vào chuỗi khách hàng để chia tuyến cho các xe tải.
 * @param pure_seq Chuỗi ID khách hàng
 * @param numTrucks Số lượng xe tải
 * @return Chuỗi đã chèn số 0
 */
vector<int> insertBalancedSeparators(const vector<int>& pure_seq, int numTrucks);

/**
 * @brief Chuyển đổi chuỗi ID khách hàng (đã có separator 0) thành Chromosome.
 *
 * Thực hiện:
 *   - Wrap: Mỗi node_id thành Gene{node_id, {}} (resupply_vector rỗng)
 *   - Chèn Depot ảo (node_id = -1): chèn 1–3 điểm ngẫu nhiên vào giữa chuỗi,
 *     không đứng cạnh nhau, không đứng ở đầu/cuối, và không kề separator.
 *
 * @param base_seq  Chuỗi ID khách hàng từ heuristic (Random, Greedy, Sweep, NN)
 * @param gen       Random engine để chọn vị trí chèn
 * @return Chromosome hoàn chỉnh (vector<Gene>) sẵn sàng đẩy vào population
 */
Chromosome buildFinalSequence(const vector<int>& base_seq, mt19937& gen);

// === REPAIR OPERATORS ===

/**
 * @brief Sửa chromosome: loại bỏ duplicate, thêm khách còn thiếu, đảm bảo P trước DL.
 *
 * Chỉ can thiệp vào các Gene có node_id > 0 (điểm khách hàng).
 * Các phần tử node_id == 0 (separator) và node_id == -1 (depot return)
 * hoàn toàn KHÔNG bị xóa, di chuyển hay thay đổi.
 * resupply_vector của mọi Gene được giữ nguyên.
 *
 * @param[in,out] seq  Chromosome cần sửa (vector<Gene>)
 * @param data         PDP instance
 * @param gen          Random engine cho repair ngẫu nhiên
 */
void quickRepairPDP(Chromosome& seq, const PDPData& data, mt19937& gen);

// === INITIALIZATION STRATEGIES ===

/**
 * @brief Tạo quần thể ban đầu đa dạng từ 4 heuristic kết hợp.
 *
 * Tỷ lệ: 10% Random, 30% Greedy Time, 30% Sweep, 30% Nearest Neighbor.
 * Mỗi cá thể được chuyển thành Chromosome qua buildFinalSequence().
 *
 * @param populationSize Số cá thể mong muốn
 * @param data           PDP instance
 * @param runNumber      Seed phụ để đảm bảo tái lập (reproducibility)
 * @return vector<Chromosome> — quần thể ban đầu
 */
vector<Chromosome> initStructuredPopulationPDP(
    int populationSize, const PDPData& data, int runNumber = 1);

/**
 * @brief Tạo quần thể bằng cách xáo ngẫu nhiên danh sách khách hàng.
 * @param populationSize Số cá thể
 * @param data           PDP instance
 * @param gen            Random engine dùng chung (seed nhất quán)
 * @return vector<Chromosome>
 */
vector<Chromosome> initRandomPDP(
    int populationSize, const PDPData& data, mt19937& gen);

/**
 * @brief Tạo quần thể theo heuristic Greedy Time (ưu tiên ready_time sớm nhất).
 * @param populationSize Số cá thể
 * @param data           PDP instance
 * @param gen            Random engine dùng chung
 * @return vector<Chromosome>
 */
vector<Chromosome> initGreedyTimePDP(
    int populationSize, const PDPData& data, mt19937& gen);

/**
 * @brief Tạo quần thể theo thuật toán Sweep (sắp xếp theo góc cực từ depot).
 * @param populationSize Số cá thể
 * @param data           PDP instance
 * @param gen            Random engine dùng chung
 * @return vector<Chromosome>
 */
vector<Chromosome> initSweepPDP(
    int populationSize, const PDPData& data, mt19937& gen);

/**
 * @brief Tạo quần thể theo heuristic Nearest Neighbor (chọn khách gần nhất kế tiếp).
 * @param populationSize Số cá thể
 * @param data           PDP instance
 * @param gen            Random engine dùng chung
 * @return vector<Chromosome>
 */
vector<Chromosome> initNearestNeighborPDP(
    int populationSize, const PDPData& data, mt19937& gen);

#endif // PDP_INIT_H
