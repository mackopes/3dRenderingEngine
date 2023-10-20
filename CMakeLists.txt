cmake_minimum_required(VERSION 3.22)
project(3DRenderingEngine)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

file(GLOB SOURCES src/*.cpp)


find_package (Eigen3 3.4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(OpenMP)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${SDL2_INCLUDE_DIRS} )

add_executable(renderer ${SOURCES})

target_include_directories(renderer
PRIVATE
${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(renderer Eigen3::Eigen)
target_link_libraries(renderer ${OpenCV_LIBS})
target_link_libraries(renderer ${SDL2_LIBRARIES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(renderer OpenMP::OpenMP_CXX)
endif()