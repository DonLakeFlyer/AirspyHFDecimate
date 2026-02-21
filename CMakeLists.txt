cmake_minimum_required(VERSION 3.16)
project(AirspyHFDecimate LANGUAGES CXX)

include(CTest)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(airspyhf_decimator
    src/main.cpp
)

target_link_libraries(airspyhf_decimator PRIVATE PkgConfig::ZeroMQ)

target_compile_options(airspyhf_decimator PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
)

if(BUILD_TESTING)
    add_executable(airspyhf_decimator_tests
        tests/test_main.cpp
    )

    target_link_libraries(airspyhf_decimator_tests PRIVATE PkgConfig::ZeroMQ)

    target_compile_options(airspyhf_decimator_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
    )

    add_test(NAME airspyhf_decimator_tests COMMAND airspyhf_decimator_tests)
endif()
