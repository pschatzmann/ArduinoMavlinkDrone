cmake_minimum_required(VERSION 3.10)
project(ArduinoMavlinkDrone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    arduino_emulator
    GIT_REPOSITORY https://github.com/pschatzmann/Arduino-Emulator.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(arduino_emulator)

file(GLOB SOURCES src/*.cpp)
add_library(ArduinoMavlinkDrone STATIC ${SOURCES})
target_include_directories(ArduinoMavlinkDrone PUBLIC src
    ${arduinoemulator_SOURCE_DIR}/src
)
target_link_libraries(ArduinoMavlinkDrone arduino_emulator)