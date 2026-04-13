cmake_minimum_required(VERSION 3.15)
project(acc_sil VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(acc SHARED src/acc.cpp)
target_include_directories(acc PUBLIC include)

add_library(acc_static STATIC src/acc.cpp)
target_include_directories(acc_static PUBLIC include)

enable_testing()
add_subdirectory(test)
