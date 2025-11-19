#pragma once
#include "solution.h" // Đảm bảo Route, VRPInstance đã được khai báo

// Chèn khách hàng vào vị trí tối ưu nhất trong tuyến
void insert_customer_best_position(Route& route, int customer_id, const VRPInstance& vrp, double alpha, double beta);

// Tối ưu hóa cục bộ tuyến bằng cách tháo/chèn lại từng khách hàng
void local_post_optimization(Route& route, const VRPInstance& vrp, double alpha, double beta);
// Thêm vào tệp insertion.h
void insert_customer_any_route(Solution& solution, int customer_id, const VRPInstance& vrp, double alpha, double beta);