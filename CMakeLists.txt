# This is what GTSAM has, so figured I might as well match their version of CMake...
cmake_minimum_required(VERSION 3.0)
project(gnss_gtsam)

# Include GTSAM
find_package(GTSAMCMakeTools REQUIRED)
include(GtsamBuildTypes)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

set(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Create the library
include_directories(BEFORE 
        ${PROJECT_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR})

message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message("PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")

# Put in all the Cpp files needed to create the library
add_library(${PROJECT_NAME} SHARED
            src/PseudoRange.cpp)   
# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
            ${INSTALL_NAME_DIR}/include)
# Link against GTSAM
target_link_libraries(${PROJECT_NAME} 
        gtsam)
# install the library (C++)
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include)

# Create the Python bindings
find_package(gtwrap REQUIRED)

# We use this as a convenience variable.
set(PYTHON_MODULE_PATH ${PROJECT_BINARY_DIR}/${PROJECT_NAME})
message("PYTHON_MODULE_PATH = ${PYTHON_MODULE_PATH}")

# Add the setup.py file
configure_file(${PROJECT_ROOT_DIR}/python/setup.py.in
               ${PYTHON_MODULE_PATH}/setup.py)

# Add the __init__.py file
configure_file(${PROJECT_ROOT_DIR}/python/__init__.py.in
               ${PYTHON_MODULE_PATH}/${PROJECT_NAME}/__init__.py)

# Add the Pybind11 template file.
configure_file(${PROJECT_ROOT_DIR}/wrap/pybind_wrapper.tpl.example
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)

# There are some files that the pybind_wrap call below expects to be there.
# At least one is a specialization/${PROJECT_NAME}.h file...  Put it where needed
configure_file(${PROJECT_ROOT_DIR}/wrap/specializations/${PROJECT_NAME}.h 
               ${PYTHON_MODULE_PATH}/specializations/${PROJECT_NAME}.h
)

set(interface_files ${PROJECT_ROOT_DIR}/gnss_gtsam.i)

pybind_wrap(${PROJECT_NAME}_py # target
            "${interface_files}" # list of interface header files
            "${PROJECT_NAME}.cpp" # the generated cpp 
            "${PROJECT_NAME}" # module_name 
            "${PROJECT_NAME}" # top namespace in the cpp file 
            "" # ignore classes
            ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
            ${PROJECT_NAME} # libs
            "${PROJECT_NAME};gtsam" # dependencies, we need the library built in step 6 as the minimum.
)

add_custom_target(python-install
    COMMAND ${PYTHON_EXECUTABLE} ${PYTHON_MODULE_PATH}/setup.py install
    DEPENDS ${PROJECT_NAME}_py
    WORKING_DIRECTORY ${PYTHON_MODULE_PATH})