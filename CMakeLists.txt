cmake_minimum_required(VERSION 3.20)
project(mppi_ca_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

add_compile_options(-lcblas)
add_compile_options(-lopenblas)

find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(ortools CONFIG REQUIRED)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})


set(XTENSOR_USE_XSIMD ON)

add_subdirectory(external/xsimd)
add_subdirectory(external/xtl)
add_subdirectory(external/xtensor)
add_subdirectory(external/xtensor-blas)
add_subdirectory(external/xtensor-python)

add_subdirectory(external/pybind11)


add_library(controller_library SHARED
        src/controller.cpp
        src/dynamics.cpp
        src/geometry.cpp
        src/orca_lines.cpp)


target_include_directories(controller_library
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        )

target_compile_definitions(controller_library PUBLIC -DUSE_CBLAS)

target_link_libraries(controller_library ${LAPACK_LIBRARIES})

target_link_libraries(controller_library xtensor)
target_link_libraries(controller_library xtensor-blas)
target_link_libraries(controller_library xsimd)


pybind11_add_module(mppi_ca_cpp python_bind/converters.hpp python_bind/converters.cpp python_bind/mppi_ca_cpp.cpp NO_EXTRAS)
target_link_libraries(mppi_ca_cpp PRIVATE xtensor-python Python::NumPy)
target_link_libraries(mppi_ca_cpp PRIVATE controller_library ortools::ortools)


add_custom_target(mppi_all)
add_dependencies(mppi_all
        controller_library
        mppi_ca_cpp)
