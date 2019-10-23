cmake_minimum_required(VERSION 3.5)

set(PYBIND11_PYTHON_VERSION 2.7 CACHE STRING "")
find_package(pybind11 QUIET)

add_compile_options(-std=c++14)

if(${pybind11_FOUND})
    
    message(STATUS "Will compile python bindings")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pycasadi_kin_dyn pyCasadiKinDyn.cpp)
    target_link_libraries(pycasadi_kin_dyn PUBLIC casadi_kin_dyn)

    
    # get python install path 
    execute_process(
        COMMAND python -c "if True:
            from distutils import sysconfig as sc
            print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Python install dir: ${PYTHON_SITE}")
    
    install(TARGETS pycasadi_kin_dyn
            DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})
            
    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")
    install(FILES  ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})

            
else()
    message(STATUS "Pybind11 not found, bindings won't be available")
endif()
