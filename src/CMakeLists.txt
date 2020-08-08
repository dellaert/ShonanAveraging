# Ceres CLI
add_executable( shonan_ceres shonan_ceres.cpp )

add_executable(pose_graph_2d pose_graph_2d.cc)
target_link_libraries(pose_graph_2d Eigen3::Eigen)
target_include_directories(pose_graph_2d PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(pose_graph_2d ${CERES_LIBRARIES})
