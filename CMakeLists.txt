cmake_minimum_required(VERSION 3.10) # Specify minimum CMake version
project(SpinStepCpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17) # Using C++17 for std::optional, std::unordered_map, etc.
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Find Eigen ---
# Option 1: If Eigen is installed system-wide or findable by CMake
find_package(Eigen3 3.3 REQUIRED NO_MODULE) # NO_MODULE uses Eigen3Config.cmake
# Option 2: If Eigen is included as a subdirectory (e.g., in a 'third_party' folder)
# add_subdirectory(third_party/eigen) # Assuming Eigen source is in third_party/eigen
# include_directories(third_party/eigen) # Or where Eigen headers are

# --- Include Directories ---
include_directories(${EIGEN3_INCLUDE_DIR}) # From find_package
# Add other include directories if your source files are organized differently
include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # To find Node.hpp, etc. if in root

# --- Source Files ---
set(SOURCES
    Node.cpp
    Orientations.cpp
    SpinStepGraph.cpp
    # Add your main.cpp or test driver cpp files here
    # e.g., main.cpp 
)

# --- Executable or Library ---
# If you're building an executable (e.g., for testing)
add_executable(SpinStepTest main.cpp ${SOURCES}) # Assuming a main.cpp for testing
target_link_libraries(SpinStepTest PRIVATE Eigen3::Eigen)

# If you plan to build this as a library to be used by other C++ projects:
# add_library(SpinStep STATIC ${SOURCES}) # Or SHARED
# target_link_libraries(SpinStep PUBLIC Eigen3::Eigen) # PUBLIC makes Eigen headers available to consumers
# target_include_directories(SpinStep PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}) # So consumers can find Node.hpp etc.

# --- Optional: Compiler Flags ---
# Example: Enable warnings
if(MSVC)
    # /W4 for MSVC
    add_compile_options(/W4)
else()
    # -Wall -Wextra -pedantic for GCC/Clang
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# --- Testing (Example with CTest if you add tests later) ---
# enable_testing()
# function(add_spinstep_test test_name test_source_file)
#     add_executable(${test_name} ${test_source_file} ${SOURCES})
#     target_link_libraries(${test_name} PRIVATE Eigen3::Eigen) # Link test executable
#     add_test(NAME ${test_name} COMMAND ${test_name})
# endfunction()
# add_spinstep_test(MyNodeTests tests/test_node.cpp) # Example