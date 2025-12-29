cmake_minimum_required(VERSION 3.20)

project(KalmanFilter
    LANGUAGES CXX
)

# Change according to convenience:
list(APPEND CMAKE_PREFIX_PATH "/home/whizbuzzer/vcpkg/packages/matplotplusplus_x64-linux/share/matplot++")
find_package(Matplot++ REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
# find_file(DATA_FOLDER data PATHS ${CMAKE_SOURCE_DIR}/data)

add_executable(KalmanFilter main.cpp)

target_compile_features(KalmanFilter PRIVATE cxx_std_17)

target_include_directories(KalmanFilter
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(KalmanFilter
    PRIVATE
        Matplot++::matplot
        ${OpenCV_LIBS}
)
