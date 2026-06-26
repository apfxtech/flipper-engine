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
    src/arm/exceptions.cpp
)
target_include_directories(arm_core PUBLIC src)

add_executable(test_alu tests/test_alu.cpp)

add_executable(conform src/conform.cpp)
target_link_libraries(conform PRIVATE arm_core)

add_executable(boot src/boot.cpp)
target_link_libraries(boot PRIVATE arm_core)

find_package(SFML 3 COMPONENTS Graphics Window System QUIET)
if(SFML_FOUND)
    add_executable(flipper_sfml src/main_sfml.cpp)
    target_link_libraries(flipper_sfml PRIVATE arm_core SFML::Graphics SFML::Window SFML::System)
    message(STATUS "SFML found: building flipper_sfml")
else()
    message(STATUS "SFML not found: skipping flipper_sfml")
endif()

enable_testing()
add_test(NAME alu COMMAND test_alu)
