# executables
set (tests SOn_test SOn_parameterization_test frobenius_prior_test shonan_factor_test shonan_ceres_test)
foreach(test ${tests})
    add_executable(${test} "${test}.cpp")
    target_link_libraries(${test} gtest_main Eigen3::Eigen ${CERES_LIBRARIES})
    target_include_directories(${test} PRIVATE ${CERES_INCLUDE_DIRS})
    add_test(NAME ${test} COMMAND ${test})
    endforeach(test)

# executables
set (executables pose_graph_2d pose_graph_3d shonan_ceres)
foreach(executable ${executables})
    add_executable(${executable} "${executable}.cc")
    target_link_libraries(${executable} Eigen3::Eigen ${CERES_LIBRARIES})
    target_include_directories(${executable} PRIVATE ${CERES_INCLUDE_DIRS})
endforeach(executable)
