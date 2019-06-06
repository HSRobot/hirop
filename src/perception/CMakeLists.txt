set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/hirop_perecption)

find_package(ecto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roscpp REQUIRED)
find_package(tf REQUIRED)
find_package(ecto_pcl REQUIRED)

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

add_subdirectory(filters)
