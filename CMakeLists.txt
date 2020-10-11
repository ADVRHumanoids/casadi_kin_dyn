cmake_minimum_required(VERSION 3.5)

project(casadi_kin_dyn VERSION 1.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(casadi 3.4.5 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(
    include/
    ${EIGEN3_INCLUDE_DIR}
    src/
)


 
# Compile and install matlogger2 library
set(LIBRARY_TARGET_NAME casadi_kin_dyn) # set library name
set(${LIBRARY_TARGET_NAME}_SRC 
        src/casadi_kin_dyn.cpp
    )
add_library(${LIBRARY_TARGET_NAME} SHARED ${${LIBRARY_TARGET_NAME}_SRC})
target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE casadi ${PINOCCHIO_LIBRARIES} ${urdfdom_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${LIBRARY_TARGET_NAME} PRIVATE -std=c++11)
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES 
        VERSION ${${PROJECT_NAME}_VERSION})
target_include_directories(${LIBRARY_TARGET_NAME} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                            ${EIGEN3_INCLUDE_DIR})

# Specify installation targets, typology and destination folders.
include(GNUInstallDirs)
install(TARGETS  ${LIBRARY_TARGET_NAME}
        EXPORT   ${LIBRARY_TARGET_NAME}
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".svn" EXCLUDE
)

include(InstallBasicPackageFiles)
install_basic_package_files(${LIBRARY_TARGET_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            EXPORT ${LIBRARY_TARGET_NAME}
                            VARS_PREFIX ${LIBRARY_TARGET_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                            
     
# Compile python bindings
add_subdirectory(bindings/python/)
                            
# Add the uninstall target
include(AddUninstallTarget)

# Add Testing target 
option(BUILD_TESTS "Compile ${PROJECT_NAME} tests" FALSE)

if(BUILD_TESTS)
  enable_testing()
  add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory(tests)
endif()



