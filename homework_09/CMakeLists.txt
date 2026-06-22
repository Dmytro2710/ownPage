cmake_minimum_required(VERSION 3.20)

project(drone_calc CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(hw9_drone_calc 
    src/main.cpp
    src/MissionProcessor.cpp
    src/solvers/AnalyticalSolver.cpp
    src/solvers/BallisticTable.cpp
    src/solvers/TableSolver.cpp
    src/providers/JsonTargetProvider.cpp
    src/config/FileConfigLoader.cpp
    src/config/ComponentFactory.cpp
    src/Geometry.cpp
    src/states/StateMoving.cpp
    src/states/StateStoped.cpp
    src/states/StateTurning.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
)

target_include_directories(hw9_drone_calc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hw9_drone_calc PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(hw9_drone_calc PRIVATE -Wall -Wextra -pedantic)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)
