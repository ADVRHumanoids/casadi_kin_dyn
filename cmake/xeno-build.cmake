find_package(Xenomai REQUIRED)

if(NOT ${Xenomai_FOUND})

    message(FATAL_ERROR "Unable to find Xenomai (you may want to set ENABLE_XENO=FALSE)")
    
else()

    add_library(${LIBRARY_TARGET_NAME}-xeno SHARED ${${LIBRARY_TARGET_NAME}_SRC})
    target_link_libraries(${LIBRARY_TARGET_NAME}-xeno PRIVATE dl)
    target_compile_definitions(${LIBRARY_TARGET_NAME}-xeno PRIVATE -DMATLOGGER2_USE_POSIX_THREAD)
    target_compile_options(${LIBRARY_TARGET_NAME}-xeno PRIVATE -std=c++14)
    set_xeno_flags(${LIBRARY_TARGET_NAME}-xeno)
    set_target_properties(${LIBRARY_TARGET_NAME}-xeno PROPERTIES 
            VERSION ${${PROJECT_NAME}_VERSION})
            
    target_include_directories(${LIBRARY_TARGET_NAME}-xeno INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

    # Specify installation targets, typology and destination folders.
    install(TARGETS  ${LIBRARY_TARGET_NAME}-xeno
            EXPORT   ${LIBRARY_TARGET_NAME}
            LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
            ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
            RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
endif()
