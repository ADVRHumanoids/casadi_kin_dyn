# set the search paths
set( Xenomai_SEARCH_PATH /usr/local/xenomai /usr/xenomai /usr/include/xenomai $ENV{XENOMAI_ROOT_DIR})

# find xeno-config.h
find_path( Xenomai_DIR
NAMES include/xeno_config.h xeno_config.h
PATHS ${Xenomai_SEARCH_PATH} )


# did we find xeno_config.h?
if( Xenomai_DIR ) 

    execute_process(COMMAND ${Xenomai_DIR}/bin/xeno-config --version OUTPUT_VARIABLE Xenomai_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE )

    message(STATUS "Xenomai ${Xenomai_VERSION} found: \"${Xenomai_DIR}\"")

    macro(set_xeno_flags target)
    
        # get compiler flags
        execute_process(COMMAND ${Xenomai_DIR}/bin/xeno-config --skin=posix --cflags 
                        OUTPUT_VARIABLE XENO_CFLAGS 
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
                        
        # get linker flags          
        get_target_property(TARGET_TYPE ${target} TYPE)
        
        if(${TARGET_TYPE} STREQUAL "SHARED_LIBRARY")
            execute_process(COMMAND ${Xenomai_DIR}/bin/xeno-config --skin=posix --ldflags --auto-init-solib
                            OUTPUT_VARIABLE XENO_LDFLAGS 
                            OUTPUT_STRIP_TRAILING_WHITESPACE)
        elseif(${TARGET_TYPE} STREQUAL "EXECUTABLE")
            execute_process(COMMAND ${Xenomai_DIR}/bin/xeno-config --skin=posix --ldflags 
                            OUTPUT_VARIABLE XENO_LDFLAGS 
                            OUTPUT_STRIP_TRAILING_WHITESPACE)
        endif()
        
        if(${Xenomai_VERSION} VERSION_LESS 3.0.0)
            set(XENO_LDFLAGS "${XENO_LDFLAGS} -lnative -lrtdm")
        endif()
        
        
        message("Target type:  ${TARGET_TYPE}")
        message("Xeno CFLAGS:  ${XENO_CFLAGS}")
        message("Xeno LDFLAGS: ${XENO_LDFLAGS}")
        
        # set flags
        set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${XENO_CFLAGS}")
        set_target_properties(${target} PROPERTIES LINK_FLAGS "${XENO_LDFLAGS}")
        
    endmacro(set_xeno_flags)

    set(Xenomai_FOUND True)
    
else( Xenomai_DIR )
    MESSAGE(STATUS "xenomai NOT found. (${Xenomai_SEARCH_PATH})")
    set(Xenomai_FOUND False)
endif( Xenomai_DIR )


