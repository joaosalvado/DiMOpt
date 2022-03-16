file(GLOB_RECURSE SOURCE_CODE mropt/*.cpp )
file(GLOB_RECURSE HEADERS_CODE mropt/*.hpp mropt/*.h )
add_library(mropt_lib STATIC ${SOURCE_CODE} ${HEADERS_CODE} mropt/StateSpace/quaternions/Quat2D.cpp mropt/StateSpace/quaternions/Quat2D.h)

target_link_libraries(mropt_lib casadi)
target_include_directories(mropt_lib PUBLIC ../)

target_link_libraries(mropt_lib
  Python3::Python
  Python3::Module
)
if(Python3_NumPy_FOUND)
  target_link_libraries(mropt_lib
    Python3::NumPy
  )
else()
  target_compile_definitions(mropt_lib WITHOUT_NUMPY)
endif()

target_include_directories(mropt_lib
        PUBLIC ${OpenCV_INCLUDE_DIRS}
        PUBLIC ${MPI_INCLUDE_DIRS})
target_link_libraries(mropt_lib
    ${OpenCV_LIBS}
        ${MPI_LIBRARIES}
)

target_link_libraries(mropt_lib Threads::Threads)

target_link_libraries(mropt_lib Matplot++::matplot)
