cmake_minimum_required(VERSION 3.16)
project(robot_system LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands so clangd (and other tools) can resolve include paths.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use RTI Connext's CMake find module. Requires NDDSHOME to be set:
#   export NDDSHOME=/path/to/rti_connext_dds-7.7.0
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{NDDSHOME}/resource/cmake")

find_package(RTIConnextDDS REQUIRED)

include(ConnextDdsCodegen)

# Generate C++11 sources from robot.idl into the build tree.
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
connextdds_rtiddsgen_run(
    LANG "C++11"
    IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/robot.idl
    OUTPUT_DIRECTORY ${GENERATED_DIR}
)

# Library target holding the generated types.
add_library(robot_types STATIC ${robot_CXX11_SOURCES})
target_include_directories(robot_types PUBLIC ${GENERATED_DIR})
target_link_libraries(robot_types PUBLIC RTIConnextDDS::cpp2_api)

set(ROBOT_APPS
    sensor_node
    control_node
    actuator_node
    telemetry_node
    telemetry_ui)

foreach(app ${ROBOT_APPS})
    add_executable(${app} src/${app}.cpp)
    target_link_libraries(${app} PRIVATE robot_types)
endforeach()
