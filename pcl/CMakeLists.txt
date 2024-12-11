cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(BUFF_PROJECT)
find_package(PCL 1.3 REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(Threads REQUIRED)

if (NOT PHO_API_CMAKE_CONFIG_PATH)
    set(PHO_API_CMAKE_CONFIG_PATH "$ENV{PHOXI_CONTROL_PATH}")
endif()

find_package(PhoXi REQUIRED CONFIG PATHS "${PHO_API_CMAKE_CONFIG_PATH}")

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS} )
link_directories(${CMAKE_SOURCE_DIR}/lib)
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test src/pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

add_executable(matrix_transform_test src/matrix_transform.cpp)
target_link_libraries(matrix_transform_test ${PCL_LIBRARIES})

add_executable(pcl_compression src/pcl_compression.cpp)
target_link_libraries(pcl_compression ${PCL_LIBRARIES})

add_executable(visual_pointcloud src/visual_pointcloud.cpp)
target_link_libraries(visual_pointcloud ${PCL_LIBRARIES})

add_executable(Pcl2Oct src/Pcl2Oct.cpp)
target_link_libraries(Pcl2Oct ${PCL_LIBRARIES} octomap octomath)

add_executable(Phoxi2Oct src/Phoxi2Oct.cpp)
target_include_directories(Phoxi2Oct PUBLIC
        ${PHOXI_INCLUDE_DIRS}
)
target_link_libraries(Phoxi2Oct
        ${PCL_LIBRARIES}
        ${PHOXI_LIBRARY}
        octomap
        octomath)

add_executable(SimulRaisim src/SimulRaisim.cpp)
target_include_directories(SimulRaisim PUBLIC
        /home/jh/Library/raisimLib/raisim/linux/include
)
target_link_directories(SimulRaisim PUBLIC
        /home/jh/Library/raisimLib/raisim/linux/lib
        )
target_link_libraries(SimulRaisim
        raisim
        Threads::Threads)

add_executable(SimulRaisimDoosan src/SimulRaisimDoosan.cpp)
target_include_directories(SimulRaisimDoosan PUBLIC
        /home/jh/Library/raisimLib/raisim/linux/include
)
target_link_directories(SimulRaisimDoosan PUBLIC
        /home/jh/Library/raisimLib/raisim/linux/lib
)
target_link_libraries(SimulRaisimDoosan
        raisim
        Threads::Threads)