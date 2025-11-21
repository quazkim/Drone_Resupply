#ifndef PDP_READER_H
#define PDP_READER_H

#include "pdp_types.h"
#include <string>
// Thêm khai báo cho hàm khoảng cách Manhattan
double manhattanDistance(double x1, double y1, double x2, double y2);
double euclideanDistance(double x1, double y1, double x2, double y2);
// Function declarations
bool readPDPFile(const string& filename, PDPData& data);
void showPDPInfo(const PDPData& data);

// Cập nhật: Hàm xây dựng CẢ HAI ma trận khoảng cách 
void buildAllDistanceMatrices(PDPData& data); 

#endif