cmake_minimum_required(VERSION 3.5)

set(PYBIND11_PYTHON_VERSION 2.7 CACHE STRING "")
find_package(pybind11 QUIET)

add_compile_options(-std=c++14)

if(${pybind11_FOUND})
    
    message(STATUS "Will compile python bindings")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    set(MODULE_NAME pycasadi_kin_dyn)
    if(${PYTHON_VERSION_MAJOR} EQUAL 3)
        set(MODULE_NAME py3casadi_kin_dyn)
    endif()

    pybind11_add_module(${MODULE_NAME} pyCasadiKinDyn.cpp)
    target_link_libraries(${MODULE_NAME} PUBLIC casadi_kin_dyn)
    target_compile_definitions(${MODULE_NAME} PRIVATE "-DCASADI_KIN_DYN_MODULE=${MODULE_NAME}")


    
    # get python install path 
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "if True:
            from distutils import sysconfig as sc
            print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Python install dir: ${PYTHON_SITE}")
    
    install(TARGETS ${MODULE_NAME}
            DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})
            
    install(FILES __init__.py DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})

            
else()
    message(STATUS "Pybind11 not found, bindings won't be available")
endif()
