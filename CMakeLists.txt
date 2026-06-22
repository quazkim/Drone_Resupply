cmake_minimum_required(VERSION 3.14)
project(DroneResupply CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type default: Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0 -Wall -Wextra")

# OR-Tools (CP-SAT backend for LSP)
find_package(ortools REQUIRED HINTS /opt/homebrew/lib/cmake/ortools)

# Shared library sources (no main)
set(PDP_SOURCES
    src/pdp_fitness.cpp
    src/pdp_ga.cpp
    src/pdp_init.cpp
    src/pdp_localsearch.cpp
    src/pdp_reader.cpp
    src/pdp_tabu.cpp
    src/pdp_utils.cpp
    src/pdp_validation.cpp
    # TS-ALNS-LSP architecture (new)
    src/pdp_lb.cpp
    src/pdp_lsp.cpp
    src/pdp_alns.cpp
    src/pdp_tabu_main.cpp
)

# --- Executable: pdp_solver ---
add_executable(pdp_solver
    src/main.cpp
    ${PDP_SOURCES}
)
target_include_directories(pdp_solver PRIVATE src)
target_link_libraries(pdp_solver PRIVATE ortools::ortools)

# Output binary vào thư mục gốc
set_target_properties(pdp_solver
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
