cmake_minimum_required(VERSION 3.2)

project(PID)

add_definitions(-std=c++11)
find_package(OpenCV 4 REQUIRED)

include_directories(include)
set(SRCS src/main.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
list(APPEND thirdparty_libraries
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(PID ${SRCS})

target_link_libraries(PID /usr/local/lib/libyaml-cpp.so ${thirdparty_libraries})
