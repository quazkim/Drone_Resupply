# PDP GA+Tabu Implementation - Code Structure

## Tổng quan
Dự án này giải quyết bài toán Pickup-Delivery Problem (PDP) với sự kết hợp xe tải và drone sử dụng thuật toán Genetic Algorithm (GA) kết hợp với Tabu Search.

## Cấu trúc code

### 1. Files chính

#### **pdp_types.h**
- Định nghĩa các cấu trúc dữ liệu cơ bản
- `PDPData`: Thông tin bài toán (nodes, distances, capacities)
- `PDPSolution`: Kết quả giải pháp (routes, costs, penalties)
- `ResupplyEvent`: Sự kiện drone resupply
- `TruckRouteInfo`: Thông tin chi tiết route của xe tải

#### **pdp_reader.cpp/h**
- Đọc file input (định dạng PDP)
- Tách distance matrix: Manhattan cho truck, Euclidean cho drone
- Parse thông tin: customers, trucks, drones, endurance, etc.

#### **pdp_init.cpp/h**
- Khởi tạo population cho GA
- 4 phương pháp:
  - **Random**: Khởi tạo ngẫu nhiên
  - **Greedy Time**: Chọn customer theo thời gian phục vụ sớm nhất
  - **Sweep**: Quét theo góc cực
  - **Nearest Neighbor**: Chọn customer gần nhất
- `initStructuredPopulationPDP()`: Tạo population đa dạng

#### **pdp_fitness.cpp/h**
- Đánh giá fitness của sequence
- `decodeAndEvaluate()`: Decode sequence thành solution
- **Greedy truck assignment**: Gán customer cho truck tốt nhất
- **Drone optimization**: Tối ưu drone resupply cho từng route
- Tính toán: timeline, costs, penalties, feasibility

#### **pdp_utils.cpp/h**
- Utility functions: print solution, statistics
- `printSolution()`: In chi tiết solution với timeline
- Format đẹp với icons và bảng

---

### 2. GA Implementation (pdp_ga.cpp/h)

#### **Crossover Operators (3 loại)**
1. **Order Crossover (OX)**
   - Copy đoạn từ parent1
   - Fill phần còn lại theo thứ tự từ parent2
   
2. **Partially Mapped Crossover (PMX)**
   - Copy đoạn từ parent2
   - Map các giá trị còn lại để tránh duplicate
   
3. **Cycle Crossover (CX)**
   - Xây dựng cycles giữa 2 parents
   - Luân phiên chọn từ parent1 và parent2

#### **Mutation Operators (3 loại)**
1. **Swap Mutation**: Đổi chỗ 2 customers ngẫu nhiên
2. **Inversion Mutation**: Đảo ngược một đoạn sequence
3. **Scramble Mutation**: Xáo trộn một đoạn sequence

#### **Selection**
- **Tournament Selection**: Chọn best trong k candidates ngẫu nhiên (k=3)

#### **Repair Operator**
- Đảm bảo mỗi customer xuất hiện đúng 1 lần
- Loại bỏ duplicate, thêm missing customers

#### **GA Main Loop**
```
1. Initialize population (diverse: Random, Greedy, Sweep, NN)
2. For each generation:
   a. Crossover: Create offspring (chọn ngẫu nhiên 1 trong 3 operators)
   b. Mutation: Mutate 10% offspring (chọn ngẫu nhiên 1 trong 3 operators)
   c. Repair: Ensure all customers present
   d. Evaluate: Decode và tính fitness
   e. Selection: 70% best offspring + 30% best parents
   f. Check improvement: Nếu 10% generations không cải thiện → Apply Tabu
3. Return best solution
```

---

### 3. Tabu Search Implementation (pdp_tabu.cpp/h)

#### **6 Move Operators**

1. **Swap Move (Type 0)**
   - Đổi chỗ 2 customers: `seq[i] ↔ seq[j]`
   - Đơn giản, hiệu quả cho local search

2. **Insert Move (Type 1)**
   - Di chuyển customer từ vị trí i đến j
   - Tốt cho thay đổi thứ tự phục vụ

3. **2-Opt Move (Type 2)**
   - Đảo ngược đoạn [i, j]: `reverse(seq[i:j+1])`
   - Classic move trong TSP/VRP

4. **2-Opt* Move (Type 3)**
   - Biến thể của 2-opt với chiến lược khác
   - Đảo ngược đoạn giữa i và j

5. **Or-Opt Move (Type 4)**
   - Di chuyển block liên tiếp (size 1-3) đến vị trí mới
   - Tốt cho tối ưu segments

6. **Relocate Pair Move (Type 5)**
   - Di chuyển cặp customers liên tiếp đến vị trí mới
   - Bảo toàn quan hệ giữa 2 customers

#### **Adaptive Weight Mechanism**

```
Weights[6] = {w0, w1, w2, w3, w4, w5}  # Khởi tạo = 1.0

Mỗi iteration:
1. Select move: Chọn move i với xác suất tỉ lệ weights[i]
2. Execute move
3. Update scores:
   - delta1 = 0.5: Nếu tạo best solution mới
   - delta2 = 0.3: Nếu cải thiện current (nhưng không phải best)
   - delta3 = 0.2: Nếu chấp nhận move xấu hơn
4. Mỗi segmentLength=100 iterations:
   weights[i] = (1-delta4)*weights[i] + delta4*(scores[i]/usedCount[i])
   với delta4 = 0.5
```

**Ý nghĩa**: Moves thành công nhiều → weight cao → được chọn nhiều hơn

#### **Tabu List Management**

```
TabuList: map<string, int>
- Key: move.key() = "type_i_j_param"
- Value: expiration_iteration = current_iter + tabu_tenure

Tabu Tenure = 0.2 * numCustomers + rand(0, 10)

Aspiration Criterion:
- Move tabu ĐƯỢC chấp nhận nếu tạo ra best solution mới
```

#### **Tabu Search Main Loop**

```
1. Initialize: currentSeq, bestSeq, weights
2. For iter in maxIterations:
   a. Select move type using adaptive weights
   b. Find best move of that type (với tabu check)
   c. Apply move
   d. Update weights and scores
   e. Add move to tabu list
   f. Update best solution
   g. Check stopping criterion
3. Return bestSeq
```

---

### 4. Main Program (main_ga_tabu.cpp)

```bash
# Usage
./main_ga_tabu <file> [pop_size] [max_gen] [mutation_rate] [run_number]

# Example
./main_ga_tabu U_30_0.5_Num_1_pd.txt 50 100 0.1 1
```

**Flow**:
1. Read instance file
2. Run GA + Tabu hybrid
3. Print detailed solution with timeline

---

## Compile & Run

### Compile
```bash
g++ -std=c++17 -Wall -g -o main_ga_tabu \
    main_ga_tabu.cpp \
    pdp_ga.cpp \
    pdp_tabu.cpp \
    pdp_init.cpp \
    pdp_fitness.cpp \
    pdp_reader.cpp \
    pdp_utils.cpp
```

### Run
```bash
# Small instance (10 customers)
./main_ga_tabu U_10_0.5_Num_1_pd.txt 30 50 0.1 1

# Medium instance (30 customers)
./main_ga_tabu U_30_0.5_Num_1_pd.txt 50 100 0.1 1

# Large instance (100 customers)
./main_ga_tabu U_100_0.5_Num_1_pd.txt 100 200 0.1 1
```

---

## Tham số quan trọng

### GA Parameters
- **Population Size**: 30-100 (phụ thuộc problem size)
- **Max Generations**: 50-200
- **Mutation Rate**: 0.1 (10% offspring)
- **Selection**: 70% offspring + 30% parents
- **Tabu Trigger**: 10% generations không cải thiện

### Tabu Parameters
- **Max Iterations**: 100 (trong GA)
- **Tabu Tenure**: 0.2 * n + rand(0,10)
- **Segment Length**: 100 (cho adaptive weights)
- **Delta weights**: δ₁=0.5, δ₂=0.3, δ₃=0.2, δ₄=0.5

---

## Điểm mạnh của implementation

1. **GA đa dạng**: 3 crossovers × 3 mutations = 9 combinations
2. **Tabu adaptive**: 6 moves với weights tự động điều chỉnh
3. **Hybrid strategy**: GA explore + Tabu exploit
4. **Structured init**: 4 phương pháp khởi tạo khác nhau
5. **Repair mechanism**: Đảm bảo solution hợp lệ
6. **Aspiration criterion**: Vượt tabu nếu tốt hơn best

---

## Output Format

### Console Output
- Initial population stats
- Generation progress với costs
- Tabu search progress
- Final solution details

### Solution Details
- Truck routes với timeline chi tiết
- Drone resupply events
- Makespan (C_max)
- Feasibility check
- Penalties (nếu có)

---

## Files cũ (deprecated)
- `pdp_ga_tabu.cpp/h`: File gộp GA+Tabu cũ (không dùng nữa)

## Files mới (hiện tại)
- `pdp_ga.cpp/h`: GA implementation độc lập
- `pdp_tabu.cpp/h`: Tabu Search với 6 moves + adaptive weights

---

## Tác giả
Implementation by Kim Quan - LabDrone/Drone_Resupply

## License
Academic/Research use
