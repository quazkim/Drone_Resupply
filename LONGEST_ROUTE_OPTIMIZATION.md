# Longest Route Optimization Strategy

## Overview
Tối ưu hóa tập trung vào **route dài nhất** trong giải pháp, vì đây thường là nút thắt xác định makespan tổng thể (C_max).

## Strategy

### Main Approach
```
1. Tìm truck route dài nhất (by total distance)
2. Áp dụng single-route operators để cải thiện nó:
   - Truck-2Opt: Đảo ngược một đoạn trong route
   - Truck-Swap: Hoán đổi 2 nodes trong route
   - Truck-Relocate: Di chuyển 1 node sang vị trí khác
3. Khi bị kẹt (no improvement), áp dụng perturbation moves
4. Lặp lại cho đến khi hội tụ
```

### Single-Route Operators (Local Search Phase)
Các phép biến đổi **chỉ ảnh hưởng đến 1 route**, không thay đổi routes khác:

| Operator | Mô tả | Ảnh hưởng |
|----------|-------|----------|
| `TRUCK_2OPT` | Đảo ngược segment [i, j] | Giảm crossing, cải thiện tour |
| `TRUCK_SWAP` | Hoán đổi nodes i và j | Thay đổi thứ tự giao hàng |
| `TRUCK_RELOCATE` | Di chuyển node sang vị trí khác | Tìm vị trí tối ưu cho node |

### Perturbation Operators (Diversification Phase)
Khi local search bị kẹt, áp dụng perturbation để **escape local optima**:

| Operator | Mô tả | Tác dụng |
|----------|-------|---------|
| `DRONE_MERGE` | Gộp 2 drone trips | Giảm số chuyến bay |
| `DRONE_SPLIT` | Tách 1 trip thành 2 | Giảm waiting time |
| `DRONE_MOVE` | Di chuyển customer giữa trips | Thay đổi cấu trúc |
| `DRONE_REASSIGN` | Đổi drone cho trip | Cân bằng tải |
| `DRONE_INSERT_INTO_TRIP` | Thêm customer vào trip | Gom nhóm tối ưu |

## Usage

### C++ Code
```cpp
#include "pdp_localsearch.h"

IntegratedLocalSearch ls(data, maxIterations = 500);

// Method 1: Original full optimization
PDPSolution result1 = ls.run(initialSolution);

// Method 2: Focus on longest route only (NEW)
PDPSolution result2 = ls.runLongestRoute(initialSolution);
```

### Main Function Example
```cpp
PDPSolution solution = ga.generateInitialSolution();

// Apply longest route optimization
IntegratedLocalSearch ls(data, 500);
solution = ls.runLongestRoute(solution);

// Print improvements
cout << "Final C_max: " << solution.totalCost << " minutes" << endl;
```

## Parameters

### In `runLongestRoute()`
```cpp
const int max_no_improve = 20;      // Trigger perturbation sau 20 iterations
const int max_perturbations = 5;    // Tối đa 5 lần perturbation
const int weight_update_freq = 15;  // Cập nhật weights mỗi 15 iterations
```

### Tuning Guide
- **max_no_improve**: Giảm để perturbation sớm hơn (tìm kiếm rộng)
- **max_perturbations**: Tăng để khám phá nhiều vùng khác nhau
- **weight_update_freq**: Giảm để thích ứng nhanh với operators hiệu quả

## Algorithm Flow

```
START
├─ Load initialSolution
├─ Current = Best = initialSolution
├─ LOOP (iteration < maxIterations):
│  ├─ Update operator weights (periodically)
│  ├─ PHASE 1: Optimize Longest Route
│  │  ├─ Find longest route
│  │  ├─ Apply single-route operators (2-opt, swap, relocate)
│  │  ├─ IF improvement: update Best & Current
│  │  └─ Reset no_improve counter
│  │
│  └─ PHASE 2: Perturbation (IF stuck)
│     ├─ IF no_improve_count >= max_no_improve:
│     │  ├─ Apply multiple perturbation moves
│     │  ├─ Recalculate times
│     │  ├─ IF better than Best: update Best
│     │  └─ Reset no_improve counter
│     └─ Continue from perturbed solution
└─ RETURN Best
```

## Performance Characteristics

### Advantages
- ✅ **Focused**: Chỉ tối ưu bottleneck route
- ✅ **Efficient**: Ít phép biến đổi, tính toán nhanh
- ✅ **Targeted**: Single-route operators rất hiệu quả
- ✅ **Flexible**: Perturbations giúp escape local optima

### When to Use
- Khi route dài nhất là nút thắt rõ ràng
- Khi có hạn chế về thời gian chạy
- Khi muốn tập trung cải thiện makespan

### Expected Improvements
- Typical: 5-15% improvement từ initial solution
- Best case: 20-30% nếu route dài nhất dễ tối ưu
- Worst case: 0-5% nếu solution đã gần optimal

## Comparison: `run()` vs `runLongestRoute()`

| Aspect | `run()` | `runLongestRoute()` |
|--------|--------|-------------------|
| **Focus** | Tất cả routes + drones | Route dài nhất |
| **Operators** | Tất cả 11 operators | Chỉ 3 single-route |
| **Complexity** | O(n²m) per iteration | O(n²) per iteration |
| **Convergence** | Chậm, toàn cục | Nhanh, địa phương |
| **Exploration** | Rộng lớn | Tập trung |
| **Best For** | Khám phá tổng quát | Tối ưu makespan |

## Example Output

```
[LONGEST ROUTE LS] Starting Longest Route Optimization...
[LONGEST ROUTE LS] Initial C_max: 450.25 minutes
[LONGEST ROUTE LS] Truck routes: 2
[LONGEST ROUTE LS] Drone trips: 3

[LONGEST ROUTE LS] Iter 0: Route 1 improved to 420.50 min
[LONGEST ROUTE LS] Iter 2: Route 1 improved to 410.75 min
[LONGEST ROUTE LS] Iter 5: Route 1 improved to 405.00 min
[LONGEST ROUTE LS] Perturbation #1 (diversification)
[LONGEST ROUTE LS] Perturbed solution C_max: 412.30 min
[LONGEST ROUTE LS] Iter 10: Route 1 improved to 400.20 min
...

[LONGEST ROUTE LS] Total improvement: 50.05 minutes (11.1%)
[LONGEST ROUTE LS] Truck improvements: 8
[LONGEST ROUTE LS] Perturbations applied: 2
```

## Implementation Details

### `findLongestRoute()`
- Tính tổng distance của mỗi route
- Trả về index của route có distance lớn nhất
- Complexity: O(m * n) với m = số trucks, n = route length

### `optimizeLongestRoute()`
- Chạy 20 iterations của single-route operators
- Adaptively chọn operator dựa trên weights
- Chỉ apply operators nếu cải thiện C_max
- Complexity: O(20 * n²)

### `applyPerturbationMove()`
- Chọn random perturbation operator
- Apply 1 lần thay đổi cấu trúc drone
- Cập nhật operator statistics
- Complexity: O(d²) với d = số drones

## Advanced Usage

### Combine with GA
```cpp
// Genetic Algorithm
GeneticAlgorithm ga(data, popSize, numGen);
PDPSolution sol = ga.run();

// Post-processing với longest route
IntegratedLocalSearch ls(data, 500);
sol = ls.runLongestRoute(sol);
```

### Multiple Runs with Restarts
```cpp
PDPSolution bestOverall = initialSol;
for (int run = 0; run < 5; ++run) {
    PDPSolution current = perturbSolution(bestOverall);  // Random start
    IntegratedLocalSearch ls(data, 200);
    current = ls.runLongestRoute(current);
    
    if (current.totalCost < bestOverall.totalCost) {
        bestOverall = current;
    }
}
```

## Troubleshooting

### Q: Improvement dừng sau vài iterations?
**A**: Route dài nhất có thể đã gần optimal. Tăng `max_no_improve` để trigger perturbations sớm hơn.

### Q: Perturbations không giúp?
**A**: Perturbation operators có thể không phù hợp. Thử tăng `max_perturbations` hoặc thay đổi cấu trúc drones thủ công.

### Q: Chậm quá?
**A**: Giảm `maxIterations` hoặc sử dụng `runLongestRoute()` thay vì `run()` (nhanh hơn 5-10x).

### Q: Local optima?
**A**: Tăng `max_perturbations` để khám phá nhiều vùng khác nhau.

## References

- **Single-Route Operators**: Vehicle Routing Problem (VRP) classical operators
- **Perturbation Strategy**: Variable Neighborhood Search (VNS) framework
- **Adaptive Weights**: Multi-Start Iterated Local Search (MSILS)
