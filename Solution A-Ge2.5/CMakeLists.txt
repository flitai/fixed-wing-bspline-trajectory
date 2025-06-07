cmake_minimum_required(VERSION 3.10)
project(FixedWingTrajectoryGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找NLopt库
find_package(NLopt REQUIRED)

# 添加可执行文件
add_executable(trajectory_generator
    main.cpp
    Bspline.cpp
    TrajectoryOptimizer.cpp
    TrajectoryGenerator.cpp
)

# 链接NLopt库
target_link_libraries(trajectory_generator PRIVATE NLopt::nlopt)

# 添加头文件目录
target_include_directories(trajectory_generator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# C++17之后需要链接数学库
if (NOT CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(trajectory_generator PRIVATE m)
endif()