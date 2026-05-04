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

add_library(ArduinoMavlinkDrone INTERFACE)
target_include_directories(ArduinoMavlinkDrone INTERFACE src)
target_link_libraries(ArduinoMavlinkDrone INTERFACE arduino_emulator)
target_compile_options(ArduinoMavlinkDrone INTERFACE -Wno-address-of-packed-member)