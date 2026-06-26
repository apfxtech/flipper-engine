cmake_minimum_required(VERSION 3.16)
project(flipper_engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

add_library(arm_core STATIC
    src/arm/exec.cpp
    src/arm/exec32.cpp
)
target_include_directories(arm_core PUBLIC src)

add_executable(test_alu tests/test_alu.cpp)

add_executable(conform src/conform.cpp)
target_link_libraries(conform PRIVATE arm_core)

enable_testing()
add_test(NAME alu COMMAND test_alu)
