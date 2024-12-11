cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(PhoXiSimple
    VERSION 1.0.0
    LANGUAGES CXX
    DESCRIPTION "Simple application to demonstrate Photoneo Device usage in Cpp code for HALCON using GenTL"
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# =================
# FIND HALCON PATHS
# =================
find_package(Halcon REQUIRED)

# print out paths
message(STATUS "   HALCON_ROOT = ${HALCON_ROOT}")
message(STATUS "   HALCON_ARCH = ${HALCON_ARCH}")

# ================
# PROJECT SETTINGS
# ================
set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

if(UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif()

add_executable(PhoXiSimple
    PhoxiSimple.cpp
)

target_link_libraries(PhoXiSimple
    PRIVATE
        HALCON::HALCONCPP
)

add_custom_command(TARGET PhoXiSimple POST_BUILD
    COMMENT "Copy halcon.dll and halconcpp.dll to target folder"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${HALCON_ROOT}/bin/${HALCON_ARCH}/halcon.dll
        ${HALCON_ROOT}/bin/${HALCON_ARCH}/halconcpp.dll
        $<TARGET_FILE_DIR:PhoXiSimple>
)
