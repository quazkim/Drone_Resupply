// vrp_parser.h
#pragma once
#include <vector>
#include <string>

struct Customer {
    int id;
    double x, y;
    int demand;
};

struct VRPInstance {
    int dimension = 0;
    int capacity = 0;
    int distance = 0; 
    int vehicle_number = 0;
    int service_time = 0; 
    std::vector<Customer> customers;
    std::vector<std::vector<double>> distance_matrix;
};

double compute_distance(const Customer& a, const Customer& b);
void compute_distance_matrix(VRPInstance& vrp);
bool read_vrp_file(const std::string& filename, VRPInstance& vrp);
bool read_vehicle_count(const std::string& instance_name, VRPInstance& vrp);
std::string get_instance_name_from_path(const std::string& filepath);
void print_vrp_info(const VRPInstance& vrp);