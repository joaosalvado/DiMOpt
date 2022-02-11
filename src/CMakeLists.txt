
set( L_EXE "")
add_executable(scp_coupled scp_coupled.cpp)
list(APPEND L_EXE scp_coupled)
add_executable(distributed_scp distributed_scp.cpp)
list(APPEND L_EXE distributed_scp)
add_executable(plot_experiments plot_experiments.cpp)
list(APPEND L_EXE plot_experiments)


FOREACH(EXE_NAME IN LISTS L_EXE)

    target_link_libraries(${EXE_NAME}  mropt_lib)

    target_include_directories(${EXE_NAME} PUBLIC ${PROJECT_INCLUDE_DIR})

    target_link_libraries(${EXE_NAME} ${MPI_LIBRARIES})

    if(MPI_COMPILE_FLAGS)
        set_target_properties(${EXE_NAME}  PROPERTIES
                COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
    endif()

    if(MPI_LINK_FLAGS)
        set_target_properties(${EXE_NAME} PROPERTIES
                LINK_FLAGS "${MPI_LINK_FLAGS}")
    endif()

    install (TARGETS ${EXE_NAME} RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH})



ENDFOREACH()




