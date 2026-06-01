# Hướng dẫn lai ghép GA cho bài toán Truck--Drone Resupply

## 1. Mã hoá solution đã chốt

Một cá thể GA được biểu diễn dưới dạng danh sách các route của truck:

```text
Solution = [Route_1, Route_2, ..., Route_n]
```

Ví dụ:

```text
Solution = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9[9,10], 8, 10, 0]
]
```

Trong đó:

| Ký hiệu | Ý nghĩa |
|---|---|
| `Route_k` | Hành trình của truck `k` |
| `0[P]` | Truck lấy tập gói `P` tại depot |
| `i` | Truck phục vụ customer `i` |
| `i[P]` | Drone resupply tập gói `P` cho truck tại node `i` |
| `0` ở cuối route | Truck kết thúc hành trình tại depot |

Ví dụ:

```text
9[9,10]
```

nghĩa là drone đưa gói `9` và `10` cho truck tại node `9`. Truck có thể phục vụ khách `9` ngay sau khi nhận hàng, sau đó tiếp tục mang gói `10` để phục vụ khách `10`.

Quy tắc quan trọng:

```text
i[P] nghĩa là drone đưa tập gói P cho truck tại node i trước khi truck phục vụ node i.
```

Do đó, với `i[P]`, mỗi gói `j ∈ P` phải thoả mãn:

```text
position(j) >= position(i)
```

trong cùng route truck.

Nghĩa là gói hàng có thể được resupply tại chính node của nó hoặc tại một node trước đó trong cùng hành trình.

---

## 2. Nguyên tắc lai ghép

Không nên lai ghép trực tiếp trên toàn bộ chuỗi có cả `0[P]` và `i[P]`, vì dễ sinh ra solution không hợp lệ.

Thay vào đó, nên thực hiện theo quy trình:

```text
Parent_1, Parent_2
    ↓
Extract customer routes
    ↓
Crossover trên thứ tự khách hàng
    ↓
Sinh child customer routes
    ↓
Kế thừa các resupply hợp lệ từ cha mẹ
    ↓
Build depot load cho các gói còn lại
    ↓
Repair solution
    ↓
Child solution
```

Lý do:

- Thứ tự phục vụ khách hàng là cấu trúc chính của route.
- Các ký hiệu `0[P]` và `i[P]` phụ thuộc vào release date, capacity, drone timing và vị trí của khách hàng.
- Nếu lai trực tiếp cả chuỗi, rất dễ bị trùng khách, mất khách, resupply sai vị trí hoặc lấy hàng trước release date.

---

## 3. Tách customer routes từ solution

Từ solution đầy đủ:

```text
Parent_1 = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9[9,10], 8, 10, 0]
]
```

Ta tách ra customer routes:

```text
CustomerRoutes_1 = [
    [1, 2, 3, 4, 5],
    [6, 7, 9, 8, 10]
]
```

Các ký hiệu bị loại bỏ khi tách customer route:

```text
0[1,2], 0[3,5], 3[4], 7[8], 9[9,10], 0
```

Chỉ giữ lại thứ tự các customer được truck phục vụ.

### Giả mã

```text
Algorithm EXTRACT_CUSTOMER_ROUTES(Solution)
Input:
    Solution
Output:
    CustomerRoutes

Begin
    CustomerRoutes ← ∅

    for each Route_k in Solution do
        R ← ∅

        for each element e in Route_k do
            if e is a customer service node then
                append customer id of e to R
            end if
        end for

        append R to CustomerRoutes
    end for

    return CustomerRoutes
End
```

---

## 4. Lai ghép một điểm cắt

### Ý tưởng

Chuyển customer routes thành chuỗi phẳng, chọn một điểm cắt, lấy phần đầu từ Parent 1, sau đó điền các customer còn thiếu theo thứ tự xuất hiện trong Parent 2.

Ví dụ:

```text
Parent_1_flat = [1, 2, 3, 4, 5, 6, 7, 9, 8, 10]
Parent_2_flat = [1, 3, 2, 5, 4, 6, 8, 7, 9, 10]
```

Chọn điểm cắt:

```text
c = 4
```

Lấy đoạn đầu từ Parent 1:

```text
Child_prefix = [1, 2, 3, 4]
```

Điền các customer còn thiếu từ Parent 2:

```text
Parent_2_flat = [1, 3, 2, 5, 4, 6, 8, 7, 9, 10]
Missing = [5, 6, 8, 7, 9, 10]
```

Child thu được:

```text
Child_flat = [1, 2, 3, 4, 5, 6, 8, 7, 9, 10]
```

Nếu giữ route length của Parent 1:

```text
route_lengths = [5, 5]
```

thì:

```text
ChildRoutes = [
    [1, 2, 3, 4, 5],
    [6, 8, 7, 9, 10]
]
```

### Giả mã

```text
Algorithm ONE_POINT_CROSSOVER
Input:
    Parent_1, Parent_2
Output:
    ChildRoutes

Begin
    R1 ← EXTRACT_CUSTOMER_ROUTES(Parent_1)
    R2 ← EXTRACT_CUSTOMER_ROUTES(Parent_2)

    F1 ← FLATTEN(R1)
    F2 ← FLATTEN(R2)

    route_lengths ← LENGTHS(R1)

    choose cut point c randomly from 1 to |F1| - 1

    ChildFlat ← F1[1 : c]

    for each customer i in F2 do
        if i not in ChildFlat then
            append i to ChildFlat
        end if
    end for

    ChildRoutes ← SPLIT_BY_LENGTHS(ChildFlat, route_lengths)

    return ChildRoutes
End
```

---

## 5. Lai ghép hai điểm cắt

### Ý tưởng

Chọn hai điểm cắt `c1` và `c2`. Child giữ đoạn giữa từ Parent 1, sau đó điền các vị trí còn trống bằng các customer còn thiếu theo thứ tự của Parent 2.

Ví dụ:

```text
Parent_1_flat = [1, 2, 3, 4, 5, 6, 7, 9, 8, 10]
Parent_2_flat = [1, 3, 2, 5, 4, 6, 8, 7, 9, 10]
```

Chọn:

```text
c1 = 3
c2 = 6
```

Giữ đoạn giữa từ Parent 1:

```text
[3, 4, 5, 6]
```

Child tạm thời:

```text
Child_flat = [_, _, 3, 4, 5, 6, _, _, _, _]
```

Các customer còn thiếu theo thứ tự Parent 2:

```text
Missing = [1, 2, 8, 7, 9, 10]
```

Điền vào các vị trí trống:

```text
Child_flat = [1, 2, 3, 4, 5, 6, 8, 7, 9, 10]
```

Chia lại theo route length:

```text
ChildRoutes = [
    [1, 2, 3, 4, 5],
    [6, 8, 7, 9, 10]
]
```

### Giả mã

```text
Algorithm TWO_POINT_CROSSOVER
Input:
    Parent_1, Parent_2
Output:
    ChildRoutes

Begin
    R1 ← EXTRACT_CUSTOMER_ROUTES(Parent_1)
    R2 ← EXTRACT_CUSTOMER_ROUTES(Parent_2)

    F1 ← FLATTEN(R1)
    F2 ← FLATTEN(R2)

    route_lengths ← LENGTHS(R1)

    choose two cut points c1 and c2 randomly
    ensure c1 < c2

    ChildFlat ← array of size |F1| filled with EMPTY

    for pos = c1 to c2 do
        ChildFlat[pos] ← F1[pos]
    end for

    MissingList ← ∅

    for each customer i in F2 do
        if i not in ChildFlat then
            append i to MissingList
        end if
    end for

    index ← 1

    for pos = 1 to |ChildFlat| do
        if ChildFlat[pos] is EMPTY then
            ChildFlat[pos] ← MissingList[index]
            index ← index + 1
        end if
    end for

    ChildRoutes ← SPLIT_BY_LENGTHS(ChildFlat, route_lengths)

    return ChildRoutes
End
```

---

## 6. Kế thừa resupply hợp lệ từ cha mẹ

Sau khi đã có `ChildRoutes`, ta cố gắng tái sử dụng các drone resupply tốt từ hai cha mẹ.

Ví dụ từ Parent 1:

```text
Parent_1 = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9[9,10], 8, 10, 0]
]
```

Các resupply candidates là:

```text
3[4]
7[8]
9[9,10]
```

Không xem `0[1,2]`, `0[3,5]`, `0[6,7]` là resupply bằng drone, vì đây là depot load.

### Giả mã trích resupply candidates

```text
Algorithm EXTRACT_RESUPPLY_TRIPS(Parent)
Input:
    Parent
Output:
    CandidateTrips

Begin
    CandidateTrips ← ∅

    for each Route_k in Parent do
        for each element e in Route_k do
            if e has form i[P] and i ≠ 0 then
                add i[P] to CandidateTrips
            end if
        end for
    end for

    return CandidateTrips
End
```

---

## 7. Điều kiện giữ lại một resupply từ cha mẹ

Giả sử một candidate là:

```text
h[P]
```

Candidate này chỉ được giữ trong child nếu thoả các điều kiện sau.

### Điều kiện 1. Node rendezvous vẫn nằm trong child route

Node `h` phải xuất hiện trong một route của child.

Ví dụ:

```text
ChildRoute = [6, 7, 9, 8, 10]
```

thì có thể xét giữ:

```text
9[9,10]
```

vì node `9` tồn tại trong child route.

### Điều kiện 2. Package được phục vụ tại hoặc sau node resupply

Với:

```text
9[9,10]
```

ta cần:

```text
position(9) >= position(9)
position(10) >= position(9)
```

Tức là khách `9` có thể được resupply tại chính node `9`, còn khách `10` được resupply trước khi truck phục vụ `10`.

Ví dụ hợp lệ:

```text
[6, 7, 9, 8, 10]
```

Ví dụ không hợp lệ cho gói `10`:

```text
[6, 10, 7, 9, 8]
```

vì khách `10` đã được phục vụ trước khi truck tới node `9`.

### Điều kiện 3. Package chưa được cung cấp trước đó

Một package chỉ nên được cung cấp một lần, thông qua một trong hai cách:

```text
0[P]      lấy tại depot
i[P]      nhận từ drone tại node i
```

Nếu package `10` đã nằm trong một depot load hoặc một resupply khác, thì không giữ thêm `9[10]`.

### Điều kiện 4. Drone trip khả thi

Với resupply:

```text
h[P]
```

cần kiểm tra:

```text
sum(q_j for j in P) <= Q_D
```

```text
drone_departure_time >= max(r_j for j in P)
```

```text
drone_arrival_time(h) <= truck_arrival_or_service_time(h) + allowed_wait
```

và drone không vi phạm endurance cũng như availability.

---

## 8. Giữ một phần của resupply candidate

Nếu một candidate không thể giữ toàn bộ, có thể giữ một phần.

Ví dụ candidate:

```text
9[9,10]
```

Child route:

```text
[6, 10, 7, 9, 8]
```

Gói `10` không hợp lệ vì đã được phục vụ trước node `9`, nhưng gói `9` vẫn hợp lệ.

Do đó có thể giữ:

```text
9[9]
```

thay vì loại bỏ toàn bộ `9[9,10]`.

### Giả mã lấy tập package hợp lệ

```text
Algorithm GET_VALID_PACKAGE_SUBSET(h[P], Child)
Input:
    h[P], Child
Output:
    P_valid

Begin
    P_valid ← ∅

    route_h ← route in Child that contains node h

    if route_h does not exist then
        return ∅
    end if

    pos_h ← position of h in route_h

    for each package j in P do
        route_j ← route in Child that contains customer j

        if route_j = route_h then
            pos_j ← position of customer j in route_h

            if pos_j >= pos_h then
                if package j has not been supplied before then
                    add j to P_valid
                end if
            end if
        end if
    end for

    return P_valid
End
```

---

## 9. Sắp xếp resupply candidates

Khi lấy candidates từ cả hai cha mẹ, có thể xảy ra trùng hoặc xung đột.

Ví dụ:

```text
Parent_1 có: 9[9,10]
Parent_2 có: 7[10]
```

Cả hai đều cấp gói `10`. Khi đó cần chọn phương án tốt hơn.

Nên ưu tiên theo các tiêu chí:

1. Resupply xuất hiện ở cả hai parent.
2. Resupply đến từ parent có fitness tốt hơn.
3. Resupply mang nhiều package hơn nhưng vẫn không vượt capacity.
4. Resupply giúp tránh truck quay về depot nhiều hơn.
5. Resupply có thời gian chờ nhỏ hơn.
6. Resupply có rủi ro infeasible thấp hơn.

### Giả mã

```text
Algorithm SORT_RESUPPLY_CANDIDATES(CandidateTrips)
Input:
    CandidateTrips
Output:
    SortedCandidateTrips

Begin
    remove exact duplicate trips

    for each trip h[P] in CandidateTrips do
        score(h[P]) = w1 * common_in_both_parents
                    + w2 * parent_fitness_priority
                    + w3 * |P|
                    + w4 * estimated_saving
                    - w5 * waiting_time
                    - w6 * feasibility_risk
    end for

    sort CandidateTrips by decreasing score

    return CandidateTrips
End
```

---

## 10. Chèn resupply vào child

Sau crossover, child ban đầu chỉ có customer routes:

```text
ChildRoutes = [
    [1, 2, 3, 4, 5],
    [6, 7, 9, 8, 10]
]
```

Khởi tạo route rỗng:

```text
Child = [
    [0, 1, 2, 3, 4, 5, 0],
    [0, 6, 7, 9, 8, 10, 0]
]
```

Sau đó chèn resupply hợp lệ:

```text
Child = [
    [0, 1, 2, 3[4], 4, 5, 0],
    [0, 6, 7[8], 9[9,10], 8, 10, 0]
]
```

Cuối cùng build depot load cho các gói chưa được drone resupply:

```text
Child = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9[9,10], 8, 10, 0]
]
```

---

## 11. Lai ghép một điểm cắt có tái sử dụng resupply

```text
Algorithm ONE_POINT_CROSSOVER_WITH_RESUPPLY_REUSE
Input:
    Parent_1, Parent_2
Output:
    Child

Begin
    R1 ← EXTRACT_CUSTOMER_ROUTES(Parent_1)
    R2 ← EXTRACT_CUSTOMER_ROUTES(Parent_2)

    F1 ← FLATTEN(R1)
    F2 ← FLATTEN(R2)

    route_lengths ← LENGTHS(R1)

    choose cut point c

    ChildFlat ← F1[1:c]

    for each customer i in F2 do
        if i not in ChildFlat then
            append i to ChildFlat
        end if
    end for

    ChildRoutes ← SPLIT_BY_LENGTHS(ChildFlat, route_lengths)

    Child ← INITIALIZE_EMPTY_ENCODED_ROUTES(ChildRoutes)

    CandidateTrips ← EXTRACT_RESUPPLY_TRIPS(Parent_1)
                    ∪ EXTRACT_RESUPPLY_TRIPS(Parent_2)

    CandidateTrips ← SORT_RESUPPLY_CANDIDATES(CandidateTrips)

    for each h[P] in CandidateTrips do
        P_valid ← GET_VALID_PACKAGE_SUBSET(h[P], Child)

        if P_valid ≠ ∅ then
            if IS_DRONE_TRIP_FEASIBLE(h[P_valid], Child) then
                INSERT_RESUPPLY(Child, h[P_valid])
                mark P_valid as supplied
                update drone schedule
            end if
        end if
    end for

    Child ← BUILD_DEPOT_LOADS_FOR_UNSUPPLIED_PACKAGES(Child)

    Child ← REPAIR_SOLUTION(Child)

    return Child
End
```

---

## 12. Lai ghép hai điểm cắt có tái sử dụng resupply

```text
Algorithm TWO_POINT_CROSSOVER_WITH_RESUPPLY_REUSE
Input:
    Parent_1, Parent_2
Output:
    Child

Begin
    R1 ← EXTRACT_CUSTOMER_ROUTES(Parent_1)
    R2 ← EXTRACT_CUSTOMER_ROUTES(Parent_2)

    F1 ← FLATTEN(R1)
    F2 ← FLATTEN(R2)

    route_lengths ← LENGTHS(R1)

    choose two cut points c1, c2 with c1 < c2

    ChildFlat ← empty array with size |F1|

    for pos = c1 to c2 do
        ChildFlat[pos] ← F1[pos]
    end for

    MissingList ← ∅

    for each customer i in F2 do
        if i not in ChildFlat then
            append i to MissingList
        end if
    end for

    index ← 1

    for pos = 1 to |ChildFlat| do
        if ChildFlat[pos] is empty then
            ChildFlat[pos] ← MissingList[index]
            index ← index + 1
        end if
    end for

    ChildRoutes ← SPLIT_BY_LENGTHS(ChildFlat, route_lengths)

    Child ← INITIALIZE_EMPTY_ENCODED_ROUTES(ChildRoutes)

    CandidateTrips ← EXTRACT_RESUPPLY_TRIPS(Parent_1)
                    ∪ EXTRACT_RESUPPLY_TRIPS(Parent_2)

    CandidateTrips ← SORT_RESUPPLY_CANDIDATES(CandidateTrips)

    for each h[P] in CandidateTrips do
        P_valid ← GET_VALID_PACKAGE_SUBSET(h[P], Child)

        if P_valid ≠ ∅ then
            if IS_DRONE_TRIP_FEASIBLE(h[P_valid], Child) then
                INSERT_RESUPPLY(Child, h[P_valid])
                mark P_valid as supplied
                update drone schedule
            end if
        end if
    end for

    Child ← BUILD_DEPOT_LOADS_FOR_UNSUPPLIED_PACKAGES(Child)

    Child ← REPAIR_SOLUTION(Child)

    return Child
End
```

---

## 13. Build depot load cho các package chưa được drone resupply

Sau khi giữ lại các drone resupply hợp lệ, các package còn lại sẽ được lấy tại depot.

Nguyên tắc:

- Truck lấy các package đã released.
- Không vượt truck capacity.
- Ưu tiên lấy package của các customer sắp được phục vụ gần nhất.
- Nếu package chưa released, truck có thể chờ hoặc quay về depot muộn hơn.

### Giả mã

```text
Algorithm BUILD_DEPOT_LOADS_FOR_UNSUPPLIED_PACKAGES(Child)
Input:
    Child
Output:
    Child with depot loads

Begin
    for each Route_k in Child do
        current_time ← 0
        current_node ← depot
        onboard ← ∅
        supplied_by_drone ← packages already assigned to drone resupply

        scan Route_k from left to right

        whenever truck is at depot do
            P ← ∅
            remaining_capacity ← Q_T - total_load(onboard)

            for each future customer j in Route_k do
                if j not in supplied_by_drone
                   and j not in onboard
                   and r_j <= current_time
                   and q_j <= remaining_capacity then

                    add j to P
                    remaining_capacity ← remaining_capacity - q_j
                end if
            end for

            replace depot node 0 by 0[P]
            onboard ← onboard ∪ P
        end whenever

        when truck reaches customer i do
            if i not in onboard then
                insert depot return 0[i] before customer i
                onboard ← onboard ∪ {i}
            end if

            serve i
            onboard ← onboard \ {i}
        end when
    end for

    return Child
End
```

---

## 14. Repair sau crossover

Sau khi crossover và kế thừa resupply, cần repair solution.

Thứ tự repair khuyến nghị:

```text
Algorithm REPAIR_AFTER_CROSSOVER
Input:
    Child
Output:
    RepairedChild

Begin
    1. Repair customer uniqueness
       Ensure each customer appears exactly once as a service node.

    2. Repair duplicated package supply
       If a package appears in multiple depot loads or resupply trips,
       keep the best supply position and remove the others.

    3. Repair invalid resupply positions
       For each h[P], remove any package j where position(j) < position(h).

    4. Repair drone capacity
       For each h[P], if total demand of P exceeds Q_D,
       split P or move some packages to another supply method.

    5. Repair drone timing
       Ensure drone can depart after release date and arrive before truck leaves h.

    6. Repair depot loads
       Ensure no package is loaded at depot before its release date.

    7. Repair truck capacity
       Ensure truck load never exceeds Q_T on any segment.

    8. Repair package availability
       Ensure package i is onboard before truck serves customer i.

    9. Final fallback
       If a drone resupply remains infeasible, replace it by depot return.

    return RepairedChild
End
```

---

## 15. Repair duplicated package supply

Một package không được cấp nhiều lần.

Ví dụ sai:

```text
[0[6,7,8], 6, 7[8], 9[9,10], 8, 10, 0]
```

Package `8` vừa được lấy tại depot, vừa được drone resupply tại node `7`.

Cách sửa:

- Nếu `7[8]` khả thi và giúp giảm depot load, giữ `7[8]`, bỏ `8` khỏi `0[6,7,8]`.
- Nếu `7[8]` không khả thi, giữ `0[6,7,8]`, bỏ `8` khỏi `7[8]`.

### Giả mã

```text
Algorithm REPAIR_DUPLICATED_PACKAGE_SUPPLY(Child)
Begin
    for each package j do
        SupplyPositions ← all positions where package j is supplied

        if |SupplyPositions| > 1 then
            best ← SELECT_BEST_SUPPLY_POSITION(j, SupplyPositions)

            for each pos in SupplyPositions do
                if pos ≠ best then
                    remove package j from pos
                end if
            end for
        end if
    end for
End
```

---

## 16. Repair invalid resupply position

Với mỗi resupply:

```text
h[P]
```

nếu có package `j ∈ P` mà customer `j` được phục vụ trước node `h`, thì phải bỏ `j` khỏi `P`.

### Giả mã

```text
Algorithm REPAIR_INVALID_RESUPPLY_POSITION(Child)
Begin
    for each Route_k in Child do
        for each resupply h[P] in Route_k do
            pos_h ← position(h)

            for each package j in P do
                pos_j ← position(j)

                if pos_j < pos_h then
                    remove j from P
                    mark j as unsupplied
                end if
            end for

            if P = ∅ then
                replace h[P] by h
            end if
        end for
    end for
End
```

---

## 17. Repair drone capacity

Nếu một resupply vượt tải drone:

```text
9[9,10,12]
```

và:

```text
q_9 + q_10 + q_12 > Q_D
```

thì cần tách hoặc bỏ bớt package.

Cách ưu tiên:

1. Giữ các package được phục vụ sớm hơn sau node `h`.
2. Giữ các package có release date phù hợp với thời điểm drone đi.
3. Giữ các package nhẹ hơn nếu cần tối đa số gói.
4. Chuyển package còn lại sang resupply khác hoặc depot return.

### Giả mã

```text
Algorithm REPAIR_DRONE_CAPACITY(Child)
Begin
    for each resupply h[P] do
        while total_demand(P) > Q_D do
            choose package j in P with lowest priority
            remove j from P
            mark j as unsupplied
        end while
    end for

    for each unsupplied package j do
        try assign j to another feasible resupply
        if failed then
            assign j to depot load or depot return
        end if
    end for
End
```

---

## 18. Repair drone timing

Với mỗi:

```text
h[P]
```

cần kiểm tra:

```text
t_drone_depart >= max(r_j for j in P)
```

```text
t_drone_arrive(h) <= t_truck_leave(h)
```

Nếu drone đến muộn:

- Cho truck chờ nếu waiting time nhỏ.
- Nếu waiting time lớn, chuyển một phần hoặc toàn bộ `P` sang node resupply sau.
- Nếu không feasible, chuyển sang depot return.

### Giả mã

```text
Algorithm REPAIR_DRONE_TIMING(Child)
Begin
    for each resupply h[P] do
        t_dep ← max(r_j for j in P)
        t_arr ← t_dep + drone_travel_time(depot, h)
        t_leave ← truck departure time from h

        if t_arr > t_leave then
            waiting_time ← t_arr - t_leave

            if waiting_time <= max_allowed_wait then
                delay truck at h
            else
                try move P to a later feasible rendezvous node

                if failed then
                    split P into smaller sets
                end if

                if still failed then
                    remove h[P]
                    mark packages in P as unsupplied
                end if
            end if
        end if
    end for
End
```

---

## 19. Repair package availability

Truck chỉ được phục vụ customer `i` nếu package `i` đã ở trên truck.

Khi scan route từ trái sang phải, duy trì:

```text
onboard = set of packages currently on truck
```

Nếu gặp customer `i` mà:

```text
i not in onboard
```

thì phải repair.

Cách sửa:

1. Thử thêm package `i` vào một resupply hợp lệ trước hoặc tại node `i`.
2. Nếu không được, insert depot return `0[i]` trước customer `i`.

### Giả mã

```text
Algorithm REPAIR_PACKAGE_AVAILABILITY(Child)
Begin
    for each Route_k in Child do
        onboard ← ∅

        scan Route_k from left to right

        if current element is 0[P] then
            onboard ← onboard ∪ P

        else if current element is h[P] then
            onboard ← onboard ∪ P

        else if current element is customer i then
            if i not in onboard then
                found ← try_add_feasible_resupply_for_package_i_before_or_at_i

                if found = false then
                    insert 0[i] before customer i
                    onboard ← onboard ∪ {i}
                end if
            end if

            serve i
            onboard ← onboard \ {i}
        end if
    end for
End
```

---

## 20. Ví dụ hoàn chỉnh

Parent 1:

```text
Parent_1 = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9[9,10], 8, 10, 0]
]
```

Parent 2:

```text
Parent_2 = [
    [0[1,3], 1, 3[4], 4, 2, 5, 0],
    [0[6], 6, 9[9,10], 10, 7[8], 8, 0]
]
```

Sau crossover trên customer sequence, giả sử thu được:

```text
ChildRoutes = [
    [1, 2, 3, 4, 5],
    [6, 7, 9, 8, 10]
]
```

Trích resupply candidates từ cha mẹ:

```text
3[4]
7[8]
9[9,10]
```

Kiểm tra trên child:

```text
Route_1 = [1, 2, 3, 4, 5]
```

`3[4]` hợp lệ vì `4` nằm sau `3`.

```text
Route_2 = [6, 7, 9, 8, 10]
```

`7[8]` hợp lệ vì `8` nằm sau `7`.

`9[9,10]` hợp lệ vì `9` nằm tại node resupply và `10` nằm sau `9`.

Child sau khi kế thừa resupply:

```text
Child = [
    [0, 1, 2, 3[4], 4, 5, 0],
    [0, 6, 7[8], 9[9,10], 8, 10, 0]
]
```

Sau khi build depot loads:

```text
Child = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9[9,10], 8, 10, 0]
]
```

---

## 21. Tóm tắt thuật toán crossover khuyến nghị

```text
Algorithm CROSSOVER_WITH_RESUPPLY_REUSE
Input:
    Parent_1, Parent_2
Output:
    Child

Begin
    1. Extract customer routes from both parents.

    2. Apply one-point or two-point crossover on customer sequences.

    3. Split child sequence into truck routes.

    4. Initialize child as routes without depot load and resupply annotations.

    5. Extract drone resupply trips from both parents.

    6. Sort resupply candidates by priority.

    7. Insert each candidate h[P] into child if:
         - h appears in child;
         - each package j in P is served at or after h;
         - package j has not been supplied before;
         - drone capacity, release date, timing and endurance are feasible.

    8. If only part of P is valid, keep the valid subset P_valid.

    9. Build depot loads for all packages not supplied by drone.

    10. Repair customer uniqueness, package supply, release date,
        truck capacity, drone capacity and drone timing.

    11. Return repaired child solution.
End
```

---

## 22. Ghi chú triển khai

Trong code, nên lưu solution bằng cấu trúc rõ ràng thay vì string thuần.

Ví dụ:

```python
solution = [
    [
        {"type": "depot", "load": [1, 2]},
        {"type": "customer", "id": 1, "resupply": []},
        {"type": "customer", "id": 2, "resupply": []},
        {"type": "depot", "load": [3, 5]},
        {"type": "customer", "id": 3, "resupply": [4]},
        {"type": "customer", "id": 4, "resupply": []},
        {"type": "customer", "id": 5, "resupply": []},
        {"type": "depot", "load": []}
    ],
    [
        {"type": "depot", "load": [6, 7]},
        {"type": "customer", "id": 6, "resupply": []},
        {"type": "customer", "id": 7, "resupply": [8]},
        {"type": "customer", "id": 9, "resupply": [9, 10]},
        {"type": "customer", "id": 8, "resupply": []},
        {"type": "customer", "id": 10, "resupply": []},
        {"type": "depot", "load": []}
    ]
]
```

Cấu trúc này giúp kiểm tra và repair dễ hơn nhiều so với parse string.

