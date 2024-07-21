add_library(my_pcd STATIC myPointCloud.cpp)

target_link_libraries(my_pcd PUBLIC Eigen3::Eigen Open3D::Open3D)

target_include_directories(my_pcd PUBLIC ${PROJECT_SOURCE_DIR})

target_compile_features(my_pcd PUBLIC cxx_std_17)
