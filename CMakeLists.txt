cmake_minimum_required(VERSION 3.16)
project(Router)

set(LIBRARY_NAME Router)

set(CMAKE_C_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE")

# 设置输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


# 包含头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 构建 Router 库（包含 router.cpp 和 parse.cpp）
add_library(Router STATIC
    src/router.cpp
    src/parse.cpp
)

target_link_libraries(${LIBRARY_NAME}
    -lrt
    -lpthread
)

# 构建主程序
add_executable(route
    src/main.cpp
)

# 链接 Router 库和系统库到主程序
target_link_libraries(route
    Router
    -lrt
    -lpthread
)