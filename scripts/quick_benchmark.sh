#!/bin/bash
# Quick benchmark runner - execute this to run quick benchmarks
# Uses sensible defaults optimized for CI/CD environments

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
MAX_CUSTOMERS=${1:-20}
SAMPLE_SIZE=${2:-30}
OUTPUT_DIR="results"

echo -e "${YELLOW}=== GA+Tabu Solver - Quick Benchmark ===${NC}"
echo "Parameters:"
echo "  Max Customers: $MAX_CUSTOMERS"
echo "  Sample Size: $SAMPLE_SIZE (None = all instances)"
echo "  Output Dir: $OUTPUT_DIR"
echo ""

# Check if executable exists
if [ ! -f main_ga_tabu ]; then
    echo -e "${RED}✗ Executable not found. Building...${NC}"
    make clean
    make
fi

# Create output directory
mkdir -p $OUTPUT_DIR

# Run benchmarks via Python script
echo -e "${YELLOW}Starting benchmark runs...${NC}\n"

python3 scripts/run_benchmarks.py \
    --max-customers $MAX_CUSTOMERS \
    --sample-size $SAMPLE_SIZE \
    --output-dir $OUTPUT_DIR

echo -e "\n${GREEN}✓ Benchmarks complete!${NC}"
echo ""
echo "Results saved to:"
ls -lt $OUTPUT_DIR/*.csv 2>/dev/null | head -2 | awk '{print "  " $NF}'
echo ""
echo "Quick summary:"
tail -n +2 ${OUTPUT_DIR}/comparison_*.csv 2>/dev/null | head -5 || true
