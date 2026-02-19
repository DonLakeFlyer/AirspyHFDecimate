cmake_minimum_required(VERSION 3.16)
project(AirspyHFDecimate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(airspyhf_decimator
    src/main.cpp
)

target_compile_options(airspyhf_decimator PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
)
