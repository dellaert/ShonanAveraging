
# cmake version to be used
cmake_minimum_required( VERSION 3.0 )

# project
project(ShonanAveraging)

# enable c++11 https://crascit.com/2015/03/28/enabling-cxx11-in-cmake/
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# For googletest:
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)  
set(GOOGLETEST_VERSION 1.10.0)
enable_testing()
include(CMakeDependentOption)
include(GNUInstallDirs)

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if (EIGEN3_FOUND)
  message("-- Found Eigen version ${EIGEN3_VERSION_STRING}: ${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_FOUND)

# Ceres
find_package(Ceres REQUIRED)
if (POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif (POLICY CMP0079)

add_subdirectory( googletest )
add_subdirectory( ceres )
add_subdirectory( src )

