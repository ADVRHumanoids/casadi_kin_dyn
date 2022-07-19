add_library(casadi_compute_distance SHARED
    f.cpp
    jacf.cpp)

target_link_libraries(casadi_compute_distance casadi_kin_dyn)

install(
    TARGETS casadi_compute_distance
    DESTINATION lib)
