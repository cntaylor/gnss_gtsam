# This is what GTSAM has, so figured I might as well match their version of CMake...
cmake_minimum_required(VERSION 3.0)
project(gnss-gtsam)

# Include GTSAM
find_package(GTSAMCMakeTools REQUIRED)
include(GtsamBuildTypes)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# Create the library
add_library(${PROJECT_NAME} SHARED
            PseudoRange.cpp)   
# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
            ${INSTALL_NAME_DIR}/include)
# Link against GTSAM
target_link_libraries(${PROJECT_NAME} 
        ${GTSAM_SHARED_LIBRARIES})

# Create the Python bindings
# Need to work through https://github.com/borglab/gtsam-project-python/blob/master/TUTORIAL.md
find_package(gtwrap)
set(interface_files ${PROJECT_SOURCE_DIR}/PseudoRange.i)

pybind_wrap(${PROJECT_NAME}_py # target
            "PseudoRange.i" # list of interface header files
            )
