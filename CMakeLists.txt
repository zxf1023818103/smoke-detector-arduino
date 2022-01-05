cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(SHIELD smokedetector_arduino)
set(CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -specs=nosys.specs")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smokedetector)

include(FetchContent)
FetchContent_Declare(
    uartframeparser
    GIT_REPOSITORY git@github.com:zxf1023818103/uartframeparser.git
    GIT_TAG origin/zephyr
)

FetchContent_MakeAvailable(uartframeparser)

include_directories(${uartframeparser_SOURCE_DIR}/include)
target_link_libraries(app LINK_PRIVATE uartframeparser)
target_sources(app PRIVATE src/main.c)
