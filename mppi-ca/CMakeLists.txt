cmake_minimum_required(VERSION 3.20)
project(mppi-ca-cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

add_compile_options(-lcblas)
add_compile_options(-lopenblas)

find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

include(FetchContent)
FetchContent_Declare(
  or-tools
  GIT_REPOSITORY https://github.com/google/or-tools.git
  GIT_TAG        main
)


set(BUILD_SHARED_LIBS OFF) # disables dynamic(shared) library building
set(BUILD_DEPS ON)

# After the following call, the CMake targets defined by OR-Tools
# will be defined and available to the rest of the build
FetchContent_MakeAvailable(or-tools)

#find_package(ortools CONFIG REQUIRED)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})


set(XTENSOR_USE_XSIMD ON)

add_subdirectory(libs/pybind11)
add_subdirectory(libs/xsimd)
add_subdirectory(libs/xtl)
add_subdirectory(libs/xtensor)
add_subdirectory(libs/xtensor-blas)
add_subdirectory(libs/xtensor-python)




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

target_link_options(controller_library PRIVATE -static-libgcc -static-libstdc++)

target_link_libraries(controller_library ${LAPACK_LIBRARIES})
target_link_libraries(controller_library xtensor)
target_link_libraries(controller_library xtensor-blas)
target_link_libraries(controller_library xsimd)
target_link_libraries(controller_library ortools::ortools)


pybind11_add_module(mppi_ca_cpp python_bind/converters.hpp python_bind/converters.cpp python_bind/mppi_ca_cpp.cpp NO_EXTRAS)

target_link_options(mppi_ca_cpp PRIVATE -static-libgcc -static-libstdc++)

target_link_libraries(mppi_ca_cpp PRIVATE xtensor-python Python::NumPy)
target_link_libraries(mppi_ca_cpp PRIVATE controller_library ortools::ortools)

set_target_properties(mppi_ca_cpp PROPERTIES PREFIX "")
set_target_properties(mppi_ca_cpp PROPERTIES OUTPUT_NAME "mppi_ca_cpp")
set_target_properties(mppi_ca_cpp PROPERTIES SUFFIX ".so")

add_custom_target(mppi_all)
add_dependencies(mppi_all
        controller_library
        mppi_ca_cpp)
