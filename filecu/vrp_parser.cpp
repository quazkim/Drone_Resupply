// vrp_parser.cpp
#include "config.h"
#include "vrp_parser.h"
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include "solution.h" // Route, Solution, VRPInstance
#include "untils.h" // evaluate_route_with_penalty
#include "insertion.h" // insert_customer_best_position, local_post_optimization
#include "vrp_parser.h" // Customer, VRPInstance
#include "solution.h" // Route, Solution

double compute_distance(const Customer& a, const Customer& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

void compute_distance_matrix(VRPInstance& vrp) {
    int n = vrp.customers.size();
    vrp.distance_matrix.assign(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            vrp.distance_matrix[i][j] = compute_distance(vrp.customers[i], vrp.customers[j]);
}

bool read_vrp_file(const std::string& filename, VRPInstance& vrp) {
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return false;
    }

    std::string line;
    enum Section { NONE, NODE_COORDS, DEMAND, DEPOT } section = NONE;
    while (std::getline(fin, line)) {
        // Xóa khoảng trắng đầu/cuối dòng
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.find("DIMENSION") != std::string::npos) {
            std::istringstream iss(line);
            std::string tmp;
            iss >> tmp >> tmp >> vrp.dimension;
        } else if (line.find("CAPACITY") != std::string::npos) {
            std::istringstream iss(line);
            std::string tmp;
            iss >> tmp >> tmp >> vrp.capacity;
        } else if (line.find("DISTANCE") != std::string::npos) {
            std::istringstream iss(line);
            std::string tmp;
            iss >> tmp >> tmp >> vrp.distance;
        } else if (line.find("SERVICE_TIME") != std::string::npos) {
            std::istringstream iss(line);
            std::string tmp;
            iss >> tmp >> tmp >> vrp.service_time;
        } else if (line.find("NODE_COORD_SECTION") != std::string::npos) {
            section = NODE_COORDS;
        } else if (line.find("DEMAND_SECTION") != std::string::npos) {
            section = DEMAND;
        } else if (line.find("DEPOT_SECTION") != std::string::npos) {
            section = DEPOT;
        } else if (vrp.distance == 0) {
            vrp.distance = std::numeric_limits<int>::max();
        } else if (vrp.capacity == 0) {
            vrp.capacity = std::numeric_limits<int>::max();
        } else if (line.find("EOF") != std::string::npos) {
            break;
        }
         else {
            std::istringstream iss(line);
            if (section == NODE_COORDS) {
                Customer cust;
                iss >> cust.id >> cust.x >> cust.y;
                vrp.customers.push_back(cust);
            } else if (section == DEMAND) {
                int id, demand;
                iss >> id >> demand;
                if (id <= vrp.customers.size())
                    vrp.customers[id - 1].demand = demand;
            } else if (section == DEPOT) {
                int depot_id;
                iss >> depot_id;
                if (depot_id == -1) section = NONE;
            }
        }
    }
    compute_distance_matrix(vrp);
    return true;
}

std::string get_instance_name_from_path(const std::string& filepath) {
    // Tìm vị trí của dấu '\' hoặc '/' cuối cùng trong đường dẫn
    size_t last_slash_pos = filepath.find_last_of("\\/");

    // Lấy phần tên file (cắt bỏ đường dẫn)
    std::string filename = (last_slash_pos == std::string::npos) ? filepath : filepath.substr(last_slash_pos + 1);

    // Tìm vị trí của dấu '.' cuối cùng để tách phần mở rộng
    size_t last_dot_pos = filename.find_last_of('.');

    // Lấy tên file không có phần mở rộng
    std::string instance_name = (last_dot_pos == std::string::npos) ? filename : filename.substr(0, last_dot_pos);

    return instance_name;
}

bool read_vehicle_count(const std::string& instance_name, VRPInstance& vrp) {
    const std::string count_file = "cmt_vehicle_counts.txt";
    std::ifstream fin(count_file);
    if (!fin) {
        std::cerr << "Error opening vehicle count file: " << count_file << std::endl;
        return false;
    }

    std::string file_id;
    int count;
    while (fin >> file_id >> count) {
        if (file_id == instance_name) {
            vrp.vehicle_number = count;
            return true;
        }
    }

    std::cerr << "Instance name not found in vehicle count file: " << instance_name << std::endl;
    return false;
}
void print_vrp_info(const VRPInstance& vrp) {

    std::cout << "Số lượng điểm (DIMENSION): " << vrp.dimension << std::endl;
    std::cout << "Tải trọng xe (CAPACITY): " << vrp.capacity << std::endl;
    std::cout << "Giới hạn quãng đường (DISTANCE): " << vrp.distance << std::endl;
    std::cout << "Thời gian phục vụ (SERVICE_TIME): " << vrp.service_time << std::endl;
    std::cout << "Số xe (vehicle_number): " << vrp.vehicle_number << std::endl;
    std::cout << "Danh sách tọa độ các điểm:" << std::endl;
    for (size_t i = 0; i < vrp.customers.size(); ++i) {
        std::cout << "  " << i << ": (" << vrp.customers[i].x << ", " << vrp.customers[i].y << "), Demand: " << vrp.customers[i].demand << std::endl;
    }
}
