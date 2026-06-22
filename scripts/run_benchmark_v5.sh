#!/usr/bin/env bash
# Benchmark V2 — TS-ALNS-LSP
# Output format matches comparison_all_10_15_20.csv:
#   Customers;Depot Location;Beta;Instance;MILP Objective;MILP CPU Time (s);GA+Tabu C_max;GA+Tabu CPU Time (s);Gap (%)

set -euo pipefail
cd "$(dirname "$0")/.."

SOLVER="./pdp_solver"
INST_DIR="./Instances 2"
WORKERS=${1:-4}
OUT_CSV="./results/comparison_v5_$(date +%Y%m%d_%H%M).csv"
TMP_DIR=$(mktemp -d)

echo "Customers;Depot Location;Beta;Instance;MILP Objective;MILP CPU Time (s);GA+Tabu C_max;GA+Tabu CPU Time (s);Gap (%)" > "$OUT_CSV"
echo "[BENCH] Output : $OUT_CSV"
echo "[BENCH] Workers: $WORKERS"

# MILP reference lookup — one entry per line: CUST DEPOT BETA NUM MILP_OBJ MILP_TIME
# depot: 0=Center 1=Border 2=Outside
MILP_DATA="
10 0 0.5 1 131.0 14.30
10 0 0.5 2 160.0 26.70
10 0 0.5 3 134.0 13.30
10 0 0.5 4 150.0 13.30
10 0 0.5 5 124.0 9.70
10 0 1.0 1 173.0 12.60
10 0 1.0 2 214.0 9.70
10 0 1.0 3 193.0 43.20
10 0 1.0 4 234.0 5.50
10 0 1.0 5 175.0 1.50
10 0 1.5 1 246.0 1.10
10 0 1.5 2 199.0 0.70
10 0 1.5 3 248.0 0.50
10 0 1.5 4 350.0 9.90
10 0 1.5 5 271.0 0.70
10 1 0.5 1 149.0 15.90
10 1 0.5 2 178.0 21.90
10 1 0.5 3 161.0 37.70
10 1 0.5 4 164.0 17.40
10 1 0.5 5 144.0 12.30
10 1 1.0 1 195.0 21.80
10 1 1.0 2 219.0 4.60
10 1 1.0 3 215.0 25.80
10 1 1.0 4 253.0 11.40
10 1 1.0 5 176.0 9.60
10 1 1.5 1 274.0 1.30
10 1 1.5 2 227.0 0.50
10 1 1.5 3 278.0 1.20
10 1 1.5 4 362.0 20.90
10 1 1.5 5 299.0 0.50
10 2 0.5 1 192.0 14.30
10 2 0.5 2 212.0 47.40
10 2 0.5 3 196.0 68.10
10 2 0.5 4 199.0 15.00
10 2 0.5 5 181.0 20.00
10 2 1.0 1 228.0 35.50
10 2 1.0 2 249.0 3.20
10 2 1.0 3 249.0 152.40
10 2 1.0 4 290.0 27.90
10 2 1.0 5 207.0 13.60
10 2 1.5 1 303.0 1.30
10 2 1.5 2 257.0 1.40
10 2 1.5 3 308.0 1.00
10 2 1.5 4 391.0 16.10
10 2 1.5 5 328.0 0.60
15 0 0.5 1 198.0 3600.20
15 0 0.5 2 163.0 3600.20
15 0 0.5 3 183.0 3600.20
15 0 0.5 4 170.0 3600.20
15 0 0.5 5 161.0 3600.50
15 0 1.0 1 256.0 3600.20
15 0 1.0 2 235.0 3606.10
15 0 1.0 3 256.0 3600.30
15 0 1.0 4 209.0 127.60
15 0 1.0 5 181.0 318.00
15 0 1.5 1 323.0 3.20
15 0 1.5 2 288.0 3600.40
15 0 1.5 3 308.0 3600.20
15 0 1.5 4 295.0 13.90
15 0 1.5 5 269.0 15.60
15 1 0.5 1 215.0 3602.70
15 1 0.5 2 185.0 3600.20
15 1 0.5 3 193.0 3600.20
15 1 0.5 4 188.0 3600.20
15 1 0.5 5 168.0 3600.20
15 1 1.0 1 255.0 3600.20
15 1 1.0 2 257.0 3600.20
15 1 1.0 3 271.0 3600.70
15 1 1.0 4 238.0 64.00
15 1 1.0 5 202.0 3606.40
15 1 1.5 1 352.0 10.00
15 1 1.5 2 311.0 73.10
15 1 1.5 3 328.0 21.70
15 1 1.5 4 323.0 13.40
15 1 1.5 5 266.0 87.50
15 2 0.5 1 250.0 3600.30
15 2 0.5 2 217.0 3600.20
15 2 0.5 3 225.0 3600.20
15 2 0.5 4 220.0 3600.20
15 2 0.5 5 208.0 3600.20
15 2 1.0 1 289.0 3600.20
15 2 1.0 2 287.0 3600.90
15 2 1.0 3 302.0 3600.60
15 2 1.0 4 268.0 32.10
15 2 1.0 5 235.0 3600.20
15 2 1.5 1 382.0 11.90
15 2 1.5 2 341.0 188.10
15 2 1.5 3 360.0 1880.30
15 2 1.5 4 353.0 50.00
15 2 1.5 5 306.0 3613.70
20 0 0.5 1 193.0 3600.30
20 0 0.5 2 202.0 3601.10
20 0 0.5 3 222.0 3605.70
20 0 0.5 4 224.0 3606.40
20 0 0.5 5 188.0 3602.60
20 0 1.0 1 288.0 3608.50
20 0 1.0 2 272.0 3602.10
20 0 1.0 3 319.0 3600.60
20 0 1.0 4 280.0 3601.60
20 0 1.0 5 275.0 3611.80
20 0 1.5 1 337.0 3600.50
20 0 1.5 2 336.0 3608.50
20 0 1.5 3 360.0 3608.20
20 0 1.5 4 397.0 3600.80
20 0 1.5 5 332.0 3601.80
20 1 0.5 1 232.0 3611.50
20 1 0.5 2 208.0 3601.20
20 1 0.5 3 220.0 3602.50
20 1 0.5 4 250.0 3605.10
20 1 0.5 5 210.0 3608.90
20 1 1.0 1 294.0 3600.80
20 1 1.0 2 324.0 3610.40
20 1 1.0 3 329.0 3600.30
20 1 1.0 4 314.0 3608.50
20 1 1.0 5 283.0 3602.40
20 1 1.5 1 381.0 3607.00
20 1 1.5 2 328.0 3602.50
20 1 1.5 3 380.0 3602.70
20 1 1.5 4 394.0 3601.00
20 1 1.5 5 346.0 3602.00
20 2 0.5 1 260.0 3610.40
20 2 0.5 2 240.0 3601.10
20 2 0.5 3 266.0 3602.20
20 2 0.5 4 262.0 3601.60
20 2 0.5 5 247.0 3600.60
20 2 1.0 1 343.0 3602.80
20 2 1.0 2 349.0 3600.40
20 2 1.0 3 367.0 3601.50
20 2 1.0 4 324.0 3609.50
20 2 1.0 5 301.0 3601.30
20 2 1.5 1 399.0 3603.10
20 2 1.5 2 381.0 3603.00
20 2 1.5 3 417.0 3602.10
20 2 1.5 4 424.0 3602.60
20 2 1.5 5 381.0 3600.40
"

DEPOT_NAME=("Center" "Border" "Outside")

# Run a single instance and write result to $TMP_DIR/<key>
run_one() {
  local cust=$1 depot=$2 beta=$3 num=$4 milp_obj=$5 milp_time=$6

  local dname="${DEPOT_NAME[$depot]}"
  local key="${cust}_${depot}_${beta}_${num}"
  local inst_file="$INST_DIR/U_${cust}_${beta}_Num_${num}.txt"

  if [[ ! -f "$inst_file" ]]; then
    echo "[SKIP] Missing: $inst_file" >&2
    return
  fi

  local lsp=3
  [[ $cust -ge 18 ]] && lsp=5

  # V3: time-based budget — 150s for n=10, 300s for n=15/20
  local tbudget=150
  [[ $cust -ge 15 ]] && tbudget=300

  local t0=$SECONDS
  local raw
  raw=$("$SOLVER" "$inst_file" --depot "$depot" --lsp "$lsp" --run 1 --time "$tbudget" 2>/dev/null || true)
  local elapsed=$(( SECONDS - t0 ))

  local our_cmax
  our_cmax=$(echo "$raw" | grep -oE 'C_max \(TS\+LSP\) : [0-9]+\.[0-9]+' | grep -oE '[0-9]+\.[0-9]+' | head -1 || true)

  if [[ -z "$our_cmax" ]]; then
    echo "[WARN] No result: $key" >&2
    echo "${cust};${dname};${beta};${num};${milp_obj};${milp_time};N/A;${elapsed}.00;N/A" > "$TMP_DIR/$key"
    return
  fi

  local gap
  gap=$(python3 -c "o=$our_cmax; m=$milp_obj; print(f'{(o-m)/m*100:.2f}%')")

  echo "${cust};${dname};${beta};${num};${milp_obj};${milp_time};${our_cmax};${elapsed}.00;${gap}" > "$TMP_DIR/$key"
  echo "[DONE] ${key} → ${our_cmax} (MILP=${milp_obj}, gap=${gap}, ${elapsed}s)"
}

export -f run_one
export SOLVER INST_DIR TMP_DIR
export DEPOT_NAME

# Generate job list and run in parallel
echo "$MILP_DATA" | grep -v '^\s*$' | \
  xargs -P "$WORKERS" -L 1 bash -c 'run_one "$@"' _

# Collect results in canonical order
echo "[BENCH] Assembling results..."
while IFS=' ' read -r cust depot beta num milp_obj milp_time; do
  [[ -z "$cust" ]] && continue
  key="${cust}_${depot}_${beta}_${num}"
  if [[ -f "$TMP_DIR/$key" ]]; then
    cat "$TMP_DIR/$key" >> "$OUT_CSV"
  fi
done <<< "$MILP_DATA"

rm -rf "$TMP_DIR"

echo ""
echo "[BENCH] Saved: $OUT_CSV"
echo "[BENCH] Summary:"
python3 - "$OUT_CSV" <<'PYEOF'
import sys, csv

rows = list(csv.DictReader(open(sys.argv[1]), delimiter=';'))
gaps = []
for r in rows:
    g = r.get('Gap (%)', 'N/A')
    try:
        gaps.append(float(g.replace('%','')))
    except:
        pass

total = len(rows)
valid = len(gaps)
avg   = sum(gaps)/valid if valid else float('nan')
mn    = min(gaps) if valid else float('nan')
mx    = max(gaps) if valid else float('nan')
better = sum(1 for g in gaps if g < 0)
print(f"  Total instances : {total}")
print(f"  Valid results   : {valid}")
print(f"  Avg gap vs MILP : {avg:.2f}%")
print(f"  Min gap         : {mn:.2f}%")
print(f"  Max gap         : {mx:.2f}%")
print(f"  Better than MILP: {better}/{valid}")

# breakdown by size
for size in [10, 15, 20]:
    sg = [float(r['Gap (%)'].replace('%','')) for r in rows
          if r['Customers'] == str(size) and r.get('Gap (%)','N/A') != 'N/A']
    if sg:
        print(f"  {size} cust avg gap  : {sum(sg)/len(sg):.2f}%")
PYEOF
