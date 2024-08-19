cmake_minimum_required(VERSION 3.12)
project(TJURM-2024)

# 设置编译指令
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)
# add_compile_options(-g -O0 -w -fno-omit-frame-pointer -Wno-notes)
add_compile_options(-g)
# set(CMAKE_BUILD_TYPE RELEASE)
# add_compile_options(-O2)

# 设置目标兵种
# add_definitions(-DTJURM_HERO)
add_definitions(-DTJURM_INFANTRY)
# add_definitions(-DTJURM_BALANCE)
# add_definitions(-DTJURM_SENTRY)
# add_definitions(-DTJURM_DRONSE)

# CUDA
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)

# OpenRM
find_package(OpenRM REQUIRED)
include_directories(${OpenRM_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV 4.5.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


# 添加项目头文件
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/json/single_include/nlohmann)


# 添加构建子目录
add_subdirectory(src/data_manager)
add_subdirectory(src/garage)
add_subdirectory(src/threads)


# 添加可执行文件
add_executable(TJURM-2024 src/main.cpp)


# 链接模块静态库
target_link_libraries(TJURM-2024
    data_manager
    garage
    threads
)

install(TARGETS TJURM-2024
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
