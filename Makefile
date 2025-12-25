CXX = clang++
CXXFLAGS = -O2 -std=c++17
SRCDIR = src
SOURCES = $(SRCDIR)/pdp_reader.cpp $(SRCDIR)/pdp_utils.cpp $(SRCDIR)/pdp_fitness.cpp $(SRCDIR)/pdp_init.cpp $(SRCDIR)/pdp_ga.cpp $(SRCDIR)/pdp_tabu.cpp $(SRCDIR)/pdp_localsearch.cpp $(SRCDIR)/main_ga_tabu.cpp
TARGET = main_ga_tabu

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@
	@echo "✓ Compiled successfully: $(TARGET)"

clean:
	rm -f $(TARGET)
	@echo "✓ Cleaned"

rebuild: clean all

test-instances2: $(TARGET)
	@python3 generate_excel_report.py 2>&1 | tee test_instances2_results.txt
