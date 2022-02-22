
set( L_EXE "")
add_executable(3robots 3robots.cpp)
list(APPEND L_EXE 3robots)


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




