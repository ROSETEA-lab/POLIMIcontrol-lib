cmake_minimum_required(VERSION 3.22)
project(POLIMIcontrol_lib)

set(CMAKE_CXX_STANDARD 20)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/CMakeModules")

# Find dependencies
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (MATIO REQUIRED)

# Generate the library
include_directories(include)

add_library(POLIMIcontrol SHARED continuous_ss.cpp differentiator.cpp discrete_derivative.cpp
        discrete_integrator.cpp discrete_ss.cpp discrete_tf.cpp FIR_filter.cpp
        util/matfile_fun.cpp controller/PID_controller.cpp controller/PIDISA_controller.cpp
        controller/AGS_controller.cpp)
target_link_libraries(POLIMIcontrol matio)

# Install the library
install(DIRECTORY include/ DESTINATION /usr/local/include/POLIMIcontrol)
install(TARGETS POLIMIcontrol EXPORT POLIMIcontrol_lib DESTINATION /usr/local/lib)

install(EXPORT POLIMIcontrol_lib FILE POLIMIcontrol.cmake DESTINATION /usr/local/lib/cmake/POLIMIcontrol)
install(FILES "CMakeModules/POLIMIcontrolConfig.cmake" DESTINATION /usr/local/lib/cmake/POLIMIcontrol)
