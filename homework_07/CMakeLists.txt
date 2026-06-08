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

add_executable(drone_calc src/main.cpp)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)

target_link_libraries(drone_calc PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(drone_calc PRIVATE -Wall -Wextra -pedantic)