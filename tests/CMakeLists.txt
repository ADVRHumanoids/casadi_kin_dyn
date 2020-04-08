cmake_minimum_required(VERSION 3.5)
include(ExternalProject)

project(casadi_kin_dyn_tests LANGUAGES C CXX VERSION 1.0.0)
add_compile_options(-std=c++11)

find_package(urdf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# compite GTest
find_package(GTest)
find_package(Threads)
if(NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main pthread)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

add_executable(test_derivatives test_derivatives.cpp PinocchioWrapper.cpp)
target_link_libraries(test_derivatives ${GTEST_BOTH_LIBRARIES} ${urdf_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_derivatives GTest-ext)
add_test(NAME test_derivatives COMMAND test_derivatives)
