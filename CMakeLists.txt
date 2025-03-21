cmake_minimum_required(VERSION 3.12)
project(AutoFlowSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find nlohmann_json package
find_package(nlohmann_json CONFIG)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2
    )
    FetchContent_MakeAvailable(json)
endif()

# Add executables
add_executable(TrafficSimulator TrafficSimulator.cpp)

# Link libraries
if(nlohmann_json_FOUND)
    target_link_libraries(TrafficSimulator PRIVATE nlohmann_json::nlohmann_json)
else()
    target_link_libraries(TrafficSimulator PRIVATE nlohmann_json::nlohmann_json)
endif()
