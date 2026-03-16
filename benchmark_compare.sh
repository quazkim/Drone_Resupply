#!/bin/bash

# Benchmark script: Run GA+Tabu on reference instances and compare with MILP optimal

echo "Drone Resupply Solver - Benchmark Comparison (GA+Tabu vs MILP)"
echo "=================================================================="

# Output file
OUTPUT="results/benchmark_ga_vs_milp.csv"
mkdir -p results

# Write header
cat > "$OUTPUT" << 'EOF'
Customers,Depot,Instance,Beta,GA_Final_Cost,MILP_Optimal,Gap_%,GA_Runtime_s
EOF

# 10 customers, all depot locations
echo ""
echo "=== Testing 10 customers ==="
for depot_name in "Center" "Border" "Outside"; do
    case "$depot_name" in
        "Center") depot_mode=0 ;;
        "Border") depot_mode=1 ;;
        "Outside") depot_mode=2 ;;
    esac
    
    for beta in 0.5 1.0 1.5; do
        for num in 1 2 3 4 5; do
            instance_file="Instances 2/U_10_${beta}_Num_${num}.txt"
            
            if [ ! -f "$instance_file" ]; then
                echo "⚠ Skipping $instance_file (not found)"
                continue
            fi
            
            echo -n "Testing: U_10_${beta}_Num_${num} (depot=$depot_name)... "
            
            # Run GA+Tabu and capture output
            start_time=$(date +%s%N)
            output=$(/usr/bin/time -f "%e" ./main_ga_tabu "$instance_file" --depot $depot_mode 2>&1)
            end_time=$(date +%s%N)
            
            # Extract final cost
            ga_cost=$(echo "$output" | grep "^Final cost:" | awk '{print $3}')
            
            if [ -z "$ga_cost" ]; then
                echo "❌ Failed to extract cost"
                continue
            fi
            
            # Query MILP optimal from CSV
            milp_optimal=$(grep "10,$depot_name,$num,0.$beta" reference/VRPRD-DR/Results/Truck-and-Drone\ \(Limited\ Drone\ Fleet\)\ MILP\ Model\ Results.csv | head -1 | awk -F',' '{print $9}')
            
            # Convert beta to format matching CSV (0.5 -> 0.5, 1.0 -> 1.0, 1.5 -> 1.5)
            beta_int=$(echo "$beta" | cut -d. -f1)
            beta_dec=$(echo "$beta" | cut -d. -f2)
            
            if [ -z "$milp_optimal" ]; then
                milp_optimal="N/A"
                gap="N/A"
            else
                gap=$(echo "scale=2; (($ga_cost - $milp_optimal) / $milp_optimal) * 100" | bc)
            fi
            
            runtime=$(echo "scale=2; ($end_time - $start_time) / 1000000000" | bc)
            
            echo "GA=$ga_cost, MILP=$milp_optimal, Gap=${gap}%"
            
            # Append to CSV
            echo "10,$depot_name,$num,$beta,$ga_cost,$milp_optimal,$gap,$runtime" >> "$OUTPUT"
        done
    done
done

echo ""
echo "=== Testing 15 customers ==="
for depot_name in "Center" "Border" "Outside"; do
    case "$depot_name" in
        "Center") depot_mode=0 ;;
        "Border") depot_mode=1 ;;
        "Outside") depot_mode=2 ;;
    esac
    
    for beta in 0.5 1.0 1.5; do
        for num in 1 2 3 4 5; do
            instance_file="Instances 2/U_15_${beta}_Num_${num}.txt"
            
            if [ ! -f "$instance_file" ]; then
                continue
            fi
            
            echo -n "Testing: U_15_${beta}_Num_${num} (depot=$depot_name)... "
            
            output=$(./main_ga_tabu "$instance_file" --depot $depot_mode 2>&1)
            ga_cost=$(echo "$output" | grep "^Final cost:" | awk '{print $3}')
            
            if [ -z "$ga_cost" ]; then
                continue
            fi
            
            milp_optimal=$(grep "15,$depot_name,$num,0.$beta" reference/VRPRD-DR/Results/Truck-and-Drone\ \(Limited\ Drone\ Fleet\)\ MILP\ Model\ Results.csv | head -1 | awk -F',' '{print $9}')
            
            if [ -z "$milp_optimal" ]; then
                milp_optimal="N/A"
                gap="N/A"
            else
                gap=$(echo "scale=2; (($ga_cost - $milp_optimal) / $milp_optimal) * 100" | bc)
            fi
            
            echo "GA=$ga_cost, MILP=$milp_optimal, Gap=${gap}%"
            echo "15,$depot_name,$num,$beta,$ga_cost,$milp_optimal,$gap,N/A" >> "$OUTPUT"
        done
    done
done

echo ""
echo "Results saved to: $OUTPUT"
echo "Done!"
