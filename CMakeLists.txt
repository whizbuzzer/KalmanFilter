cmake_minimum_required(VERSION 3.10)
project(
    KalmanFilter
)
# add_executable(
#     KalmanFilter
#     main.cpp
#     KalmanFilter.h
# )

list(APPEND CMAKE_PREFIX_PATH "/home/whizbuzzer/vcpkg/packages/matplotplusplus_x64-linux/share/matplot++")
find_package(Matplot++ REQUIRED)

# So that we don't have to mention them in the "#include" line:
set(INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (Matplot++_FOUND)
    add_executable(KalmanFilter main.cpp)
    target_link_libraries(KalmanFilter PUBLIC Matplot++::matplot)
endif ()
