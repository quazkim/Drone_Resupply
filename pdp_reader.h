#ifndef PDP_READER_H
#define PDP_READER_H

#include "pdp_types.h"
#include <string>
double euclideanDistance(double x1, double y1, double x2, double y2);
// Function declarations
bool readPDPFile(const string& filename, PDPData& data);
void showPDPInfo(const PDPData& data);

// Cập nhật: Hàm xây dựng ma trận khoảng cách chung
void buildDistanceMatrix(PDPData& data); 
// Xóa getCustomerNodes (vì isCustomer đã tốt hơn)
// Xóa buildDistanceMatrix (vì buildAll... thay thế)

#endif