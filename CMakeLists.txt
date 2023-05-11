cmake_minimum_required(VERSION 3.10)
project(
    KalmanFilter
)

# Change according to convenience:
list(APPEND CMAKE_PREFIX_PATH "/home/whizbuzzer/vcpkg/packages/matplotplusplus_x64-linux/share/matplot++")
find_package(Matplot++ REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS})
# find_file(DATA_FOLDER data PATHS ${CMAKE_SOURCE_DIR}/data)

if (Matplot++_FOUND)
    add_executable(KalmanFilter main.cpp)
    target_link_libraries(KalmanFilter PUBLIC Matplot++::matplot ${OpenCV_LIBS})
    target_include_directories(KalmanFilter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif ()

