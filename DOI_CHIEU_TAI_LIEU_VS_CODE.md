# Đối chiếu tài liệu MD vs code hiện tại (Init Solution & GA)

Ngày tổng hợp: 2026-05-24

## 0) Mục tiêu
Bạn có 2 file hướng dẫn (spec) và muốn biết code hiện tại phần **khởi tạo lời giải (init solution)** và **GA** đang **đúng phần nào / chưa đúng phần nào** so với spec, để có cái nhìn khách quan.

## 1) Tài liệu và code đã đọc

### 1.1 Spec (Markdown)
- ga_encoding_init_solution.md: mô tả encoding dạng danh sách route của truck với ký hiệu `0[P]`, `i`, `i[P]`, và quy trình tạo nghiệm ban đầu.
- ga_crossover_resupply_reuse.md: mô tả crossover “đúng hướng” (crossover trên thứ tự customer trước, sau đó tái sử dụng resupply hợp lệ + build depot load + repair).

### 1.2 Code (C++ trong src)
Các file chính liên quan init + GA + decode/fitness + validation:
- src/pdp_types.h: định nghĩa `Gene/SeqStop` và encoding hiện tại.
- src/pdp_init.h / src/pdp_init.cpp: tạo population ban đầu và build chromosome.
- src/pdp_ga.h / src/pdp_ga.cpp: GA, crossover, mutation, repairSequence.
- src/pdp_fitness.h / src/pdp_fitness.cpp: `decode_sequence()` và kiểm tra/penalty (truck load, drone endurance, drone capacity theo số gói, …).
- src/pdp_validation.cpp: kiểm tra logic (bao gồm resupply ordering theo segment).
- src/pdp_utils.cpp: validatePDPConstraints() (precedence + resupply ordering) và print.

## 2) Nhận xét quan trọng trước khi so sánh

### 2.1 Spec và code đang không cùng “một encoding”
Spec định nghĩa lời giải theo dạng:

- `Solution = [Route_1, Route_2, ..., Route_n]`
- Mỗi `Route_k` chứa các phần tử kiểu `0[P]` (depot load), `i` (serve customer i), `i[P]` (drone resupply tại node i)

Trong khi code hiện tại dùng một **chuỗi gene tuyến tính**:

- `Chromosome = vector<Gene>`
- `Gene.node_id > 0`: customer node
- `Gene.node_id == 0`: separator chia truck 1 / truck 2
- `Gene.node_id == -1`: “depot return command” (quay về depot ảo)
- `Gene.resupply_vector`: list các “package/customer IDs” mà drone mang đến tại node này

Vì vậy, khi nói “đúng/chưa đúng” cần hiểu theo 2 mức:

1) **Đúng về ý tưởng/logic** (có kiểm tra release date, có mô hình drone rendezvous, …)
2) **Đúng theo spec encoding** (có thực sự biểu diễn `0[P]`, có quy trình tái sử dụng resupply sau crossover, …)

## 3) Mapping nhanh giữa spec và code

| Thành phần | Spec | Code hiện tại | Nhận xét |
|---|---|---|---|
| Nhiều truck | danh sách nhiều route | 1 chromosome, `0` là separator | Code hiện đang “hardcode” gần như cho 2 truck (1 separator) |
| Depot load `0[P]` | biểu diễn rõ set package P ở depot (có thể nhiều lần) | `-1` là lệnh quay về depot, nhưng không có `P` (không lưu package load set) | Không tương đương 1-1 với spec |
| Drone resupply `i[P]` | node i nhận P trước khi truck phục vụ i | `resupply_vector` tại node i tạo 1 drone event giao packages | Code có mô hình rendezvous, nhưng semantics “trước khi phục vụ i” không khớp hoàn toàn (xem mục 4.4) |
| Ràng buộc ordering `position(pkg) >= position(i)` | bắt buộc trong cùng route | Có validation ở pdp_validation.cpp / pdp_utils.cpp | Nhưng GA/decoder không luôn enforce như repair cứng; chủ yếu dựa penalty/validation |

## 4) Đối chiếu phần INIT SOLUTION

### 4.1 Spec yêu cầu (tóm tắt)
Theo ga_encoding_init_solution.md, init nên theo “layered approach”:
1) Generate customer sequence
2) Split customers cho trucks
3) Construct từng truck route
4) Insert depot returns và drone resupply
5) Repair

Và các feasibility quan trọng: uniqueness, package availability, release date, truck capacity, drone capacity, drone timing, endurance.

### 4.2 Code đang làm gì
- initStructuredPopulationPDP() sinh quần thể bằng 4 heuristic: Random / GreedyTime / Sweep / NearestNeighbor.
- buildFinalSequence() tạo 2 route bằng cách split theo tỉ lệ ngẫu nhiên 0.4–0.6 (có cố gắng giữ cặp P–DL cùng xe), rồi ghép lại bằng 1 separator (node_id=0).
- Trong processRoute() (buildFinalSequence), code chỉ cân nhắc drone vs depot-return chủ yếu khi gặp node type `D` với `ready_time > 0`.

### 4.3 Phần khớp spec (đúng hướng)
- Có nhiều chiến lược init để tạo đa dạng (spec cũng khuyến nghị đa dạng).
- Có xét “release/ready time” khi quyết định drone (`d_start = max(drone_avail, ready_time)`), và decode_sequence/buildDroneEvent dùng `max_ready` cho nhiều gói.
- Có xét drone endurance (so flight time với droneEndurance).
- Có xét truck capacity (dựa current_load và truckCapacity) và có depot-return khi overload.

### 4.4 Các điểm lệch spec / chưa đúng (quan trọng)

#### (A) Không có depot load `0[P]` đúng nghĩa
- Spec yêu cầu `0[P]` (set package được lấy ở depot, có thể nhiều lần).
- Code chỉ có gene `-1` để “quay về depot” và reset tải, nhưng không lưu “P” đã lấy.

Hệ quả: khó đối chiếu đúng các bước như `SELECT_DEPOT_LOAD`, `BUILD_DEPOT_LOADS_FOR_UNSUPPLIED_PACKAGES`, và repair liên quan “lấy trước release date”.

#### (B) Batching resupply trong init có nguy cơ vi phạm ordering `position(pkg) >= position(node_resupply)`
Trong buildFinalSequence/processRoute():
- Khi gặp D node và chọn drone, code `pending_resupply.push_back(node)`.
- Khi batch đầy hoặc flush, code gán `resupply_vector = pending_resupply` tại **node cuối của batch**.

Điều này dễ tạo tình huống:
- Route: ... A (D), B (D), ...
- pending_resupply = [A, B]
- resupply_vector được gán tại node B

=> package A được drone giao tại node B, nhưng A đã đứng trước B trong route.
Điều này **trái với rule spec** (và cũng trái với validation “resupply ordering” mà bạn đã viết ở pdp_validation.cpp).

#### (C) Spec cho phép “resupply tại chính node của nó” (i có thể nằm trong P), nhưng decode hiện tại không hỗ trợ đúng semantics này
Spec nói gói có thể resupply tại chính node của nó (truck nhận trước khi phục vụ node đó).

Trong decode_sequence() hiện tại:
- Bước 1 làm “Delivery-first” chỉ giao nếu cargo đã có.
- Bước 2 nhận drone resupply (tăng tải, add cargo).
- Với node type `D`, nếu chưa delivered ở bước 1 thì bước 3 chỉ “coi như thăm thông thường”, không giao lại sau khi nhận.

Hệ quả: nếu `resupply_vector` tại node i có chứa i, thì i **không được giao ngay tại i** (vì đã qua bước delivery).
Điều này lệch spec.

#### (D) Drone capacity trong code là “số gói” thay vì “tổng khối lượng”
Spec: `sum(q_j) <= Q_D`.
Code: check `(int)resupply_vector.size() <= data.getDroneCapacity()`.

Nếu bài toán thật của bạn cần capacity theo trọng lượng, hiện tại code chưa đúng theo spec.

#### (E) Split khách cho trucks trong init khác spec
Spec mô tả split dựa “estimated completion time”.
Code split theo ratio 0.4–0.6 (ngẫu nhiên) + cố giữ P/DL cùng xe.

Điểm này không sai nếu bạn muốn đa dạng, nhưng **khác tài liệu**.

## 5) Đối chiếu phần GA (crossover / mutation / repair)

### 5.1 Spec yêu cầu (tóm tắt)
Theo ga_crossover_resupply_reuse.md:
- Không crossover trực tiếp trên chuỗi có cả depot load & resupply annotation.
- Làm theo pipeline:
  1) Extract customer routes
  2) Crossover trên thứ tự customer
  3) Sinh child customer routes
  4) Kế thừa resupply hợp lệ (lọc + có thể giữ subset)
  5) Build depot load cho phần còn lại
  6) Repair theo thứ tự (uniqueness, duplicated supply, invalid position, drone cap, timing, depot load release date, truck cap, package availability, …)

### 5.2 Code đang làm gì
- GA dùng encoding gene-based (Chromosome vector<Gene>) và có 4 crossover: one-point, OX, PMX, CX.
- onePointCrossover() và orderCrossover() thao tác trên toàn bộ gene sequence (bao gồm cả 0/-1) và copy cả Gene (kèm resupply_vector).
- pmxCrossover() và cycleCrossover() có bước splitChromosome(): tách customer genes và “specials” (node_id<=0), sau đó reassemble theo skeleton specials của parent1.
- repairSequence() sau crossover/mutation:
  - đảm bảo mỗi customer xuất hiện đúng 1 lần,
  - đảm bảo node_id==0 xuất hiện đúng 1 lần,
  - precedence P trước DL.
  - KHÔNG sửa resupply_vector.

### 5.3 Phần khớp spec (đúng hướng)
- Có cố gắng tách customer genes khỏi special nodes trong PMX/CX (gần với “crossover trên customer order”).
- Có repair “uniqueness” sau crossover/mutation.

### 5.4 Các điểm lệch spec / chưa đúng (quan trọng)

#### (A) Crossover vẫn “động vào” special nodes và resupply theo cách spec khuyên tránh
- onePointCrossover và OX cho phép 0/-1 (và resupply_vector) đi theo gene, không theo pipeline “extract → crossover customers → rebuild/resupply reuse”.
- Điều này dễ tạo child có resupply không hợp lệ (sai ordering, sai segment, duplicated supply).

#### (B) Không có bước “resupply reuse” đúng như spec
Spec có hẳn các bước:
- extract trips `h[P]` từ parent
- sort candidates
- lọc theo điều kiện (node tồn tại, ordering, chưa supplied, feasible)
- giữ subset P_valid
- update drone schedule

Code hiện tại không có pipeline này; resupply chỉ được “kế thừa ngẫu nhiên” vì Gene được copy qua crossover.

#### (C) repairSequence() không sửa resupply_vector, nên tạo nhiều trạng thái sai
Một số hệ quả trực tiếp:
- repairSequence có thể đổi `g.node_id` (để thay duplicate bằng missing customer) nhưng giữ nguyên resupply_vector của gene đó.
  => resupply_vector có thể tham chiếu package/customer không liên quan hoặc không nằm trong chromosome.
- repairSequence ép “chỉ 1 separator”, trong khi data.numTrucks có thể >2 (validateSolution kỳ vọng numTrucks-1 separators). Nếu mở rộng >2 truck, repair sẽ làm sai encoding.

#### (D) Mutation droneResupplyMutation thiếu ràng buộc segment và dedup
- droneResupplyMutation chọn một pkg từ resupply_vector rồi chuyển sang node bất kỳ có index < pkg_pos.
- Không kiểm tra “cùng truck segment” (không vượt separator).
- Không kiểm tra pkg đã tồn tại ở resupply_vector khác.

#### (E) Không có repair cho “duplicated package supply” như spec
Spec yêu cầu package chỉ supplied 1 lần.
Code không có bước xóa duplicate package trong các resupply_vector.

Ngoài ra, trong decode_sequence(), vòng lặp add load:
- `truck.current_load += data.demands[pid]; cargo_on_truck.insert(pid);`

Nếu cùng một pid bị cấp nhiều lần, set sẽ chỉ lưu 1 lần nhưng current_load vẫn cộng nhiều lần.
=> có thể tạo penalty/overload “giả” (không đúng mô hình supply 1 gói).

## 6) Đối chiếu feasibility/repair theo từng ràng buộc trong spec

| Ràng buộc/Repair (spec) | Có trong code? | Mức độ | Ghi chú |
|---|---:|---|---|
| Customer uniqueness | Có | Khá đúng | repairSequence + eval coverage |
| Package availability (phải có hàng trước khi serve) | Một phần | Chưa đúng theo spec | Với type D hiện tại, nếu không có cargo vẫn “thăm”, không buộc delivery |
| Release date (depot load/resupply sau release) | Có phần drone | Tương đối | buildDroneEvent dùng max readyTimes; depot-load không biểu diễn rõ |
| Drone capacity | Có | Khác mô hình | Code dùng số gói, spec dùng tổng demand |
| Drone endurance | Có | Đúng hướng | buildDroneEvent + penalty |
| Drone timing / truck wait | Có | Đúng hướng | truck chờ tới resupply_end_time |
| Invalid resupply position (pos(pkg) < pos(node)) | Có validator, không repair | Thiếu | pdp_validation.cpp check nhưng GA không auto repair |
| Duplicated package supply | Không | Thiếu | spec yêu cầu repair |
| Build depot loads cho unsupplied | Không (theo encoding spec) | Thiếu | code dùng -1 depot return, không có `0[P]` |

## 7) Kết luận ngắn gọn
- Code hiện tại **có mô hình GA + decode drone rendezvous**, nhưng đang dùng **một encoding khác** (gene sequence với separator/depot-return) so với encoding `0[P]`, `i[P]` trong 2 file MD.
- Một số ý tưởng spec đã có trong code (đa chiến lược init, check readyTime/release cho drone, endurance, truck capacity).
- Tuy nhiên, các phần “đặc thù” của spec crossover (crossover trên customer order + resupply reuse hợp lệ + build depot load + repair theo thứ tự) **chưa được implement**.
- Init hiện tại có rủi ro lớn vi phạm rule ordering `position(pkg) >= position(resupply_node)` do batching pending_resupply vào node cuối.

## 8) Gợi ý hướng đi (để bạn quyết định)
Bạn có 2 lựa chọn rõ ràng:

1) **Bám đúng spec MD** (encoding `0[P]`, `i[P]`) và refactor code theo pipeline trong tài liệu.
2) **Giữ encoding hiện tại** (Chromosome + resupply_vector + -1 depot return) và cập nhật lại 2 file MD để phản ánh đúng mô hình đang chạy.

Nếu bạn chọn (1), các việc ưu tiên cao nhất để “đúng spec”:
- Sửa init batching để `resupply_vector` tại node i chỉ chứa packages của các node nằm sau (hoặc chính i nếu bạn muốn hỗ trợ resupply-at-self, thì cần sửa decode để giao sau khi nhận).
- Thêm repair sau crossover cho resupply: ordering, segment, dedup, drone cap/timing.
- Không crossover trực tiếp resupply/depot-return; crossover chỉ trên customer order rồi tái dựng resupply theo spec.

Nếu bạn chọn (2), tối thiểu nên:
- Enforce resupply ordering trong repair hoặc trong decode (phạt nặng/loại bỏ) để tránh init/GA sinh nghiệm “logic sai”.
- Deduplicate packages trong resupply_vector và tránh double-add load trong decode.
- Đồng bộ logic số separator với data.numTrucks.
