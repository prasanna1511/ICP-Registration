cmake_minimum_required(VERSION 3.16)
PROJECT(project)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED HINTS ${PROJECT_SOURCE_DIR}/Open3D/lib/cmake)

# add_subdirectory(myPointCloud)
# add_subdirectory(main)
add_subdirectory(src)


add_executable(main main.cpp)
target_include_directories(main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(main PUBLIC cxx_std_20)
target_link_libraries(main PUBLIC my_pcd)
