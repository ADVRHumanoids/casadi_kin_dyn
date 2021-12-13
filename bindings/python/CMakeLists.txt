cmake_minimum_required(VERSION 3.5)

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

    # Python package deployment
    add_custom_target(generate_python_package
        COMMAND mkdir -p ${PROJECT_NAME}
        COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}
        COMMAND ./copy_external_dependencies.py $<TARGET_FILE:${MODULE_NAME}> -d ${PROJECT_NAME} -c
        COMMAND python3 -m build
        DEPENDS ${MODULE_NAME}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating package ${MODULE_NAME}"
    )

    add_custom_target(upload_python_package
        COMMAND twine upload dist/*
        DEPENDS generate_python_package
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Uploading package ${MODULE_NAME}"
    )
      
else()
    message(STATUS "Pybind11 not found, bindings won't be available")
endif()
