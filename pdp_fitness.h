#ifndef PDP_FITNESS_H
#define PDP_FITNESS_H

#include "pdp_types.h"
#include <vector>

/**
 * @brief Hàm đánh giá fitness cho PDP
 * Xử lý sequence (thứ tự khách hàng), gán từng khách cho xe rảnh nhất
 * @param seq Chromosome - thứ tự phục vụ khách hàng
 * @param data Dữ liệu bài toán PDP
 * @return PDPSolution chứa totalCost (C_max) và totalPenalty
 */
PDPSolution decodeAndEvaluate(const std::vector<int>& seq, const PDPData& data);

#endif // PDP_FITNESS_H
