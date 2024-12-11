#[=======================================================================[.rst:
FindHalcon
---------

Locate the MVTec HALCON Machine Vision Software.

Imported targets
^^^^^^^^^^^^^^^^

This module defines the following :prop_tgt:`IMPORTED` targets:

``HALCON::HALCON``
  HALCON ``halcon`` library, if found
``HALCON::HALCONXL``
  HALCON ``halconxl`` library, if found
``HALCON::HALCONC``
  HALCON ``halconc`` library, if found
``HALCON::HALCONCXL``
  HALCON ``halconcxl`` library, if found
``HALCON::HALCONCPP``
  HALCON ``halconcpp`` library, if found
``HALCON::HALCONCPPXL``
  HALCON ``halconcppxl`` library, if found


Result variables
^^^^^^^^^^^^^^^^

This module will set the following variables in your project:

``HALCON_FOUND``
  Found the MVTec HALCON Machine Vision Software
``HALCON_INCLUDE_DIR``
  the directory containing the HALCON headers
``HALCON_ROOT``
  The root directory of the MVTec HALCON Machine Vision Software installation
``HALCON_ARCH``
  The architecture of the MVTec HALCON Machine Vision Software
``MVTEC_HALCON_DOTNET_LIBRARY``
  The HALCON halcondotnet library path


The library variables below are set as normal variables.  These
contain debug/optimized keywords when a debugging library is found.

``HALCON_LIBRARY``
  The HALCON ``halcon`` library
``HALCONXL_LIBRARY``
  The HALCON ``halconxl`` library
``HALCONC_LIBRARY``
  The HALCON ``halconc`` library
``HALCONCXL_LIBRARY``
  The HALCON ``halconcxl`` library
``HALCONCPP_LIBRARY``
  The HALCON ``halconcpp`` library
``HALCONCPPXL_LIBRARY``
  The HALCON ``halconcppxl`` library

Hints
^^^^^

``HALCONROOT`` is an environment variable that would correspond to the installation folder
``HALCONARCH`` is an environment variable that would correspond to the architecture


Example usage
^^^^^^^^^^^^^

::

    find_package(Halcon REQUIRED)

    add_executable(foo1 foo1.cpp)
    target_link_libraries(foo1 HALCON::HALCON)

    add_executable(fooxl1 fooxl1.cpp)
    target_link_libraries(fooxl1 HALCON::HALCONXL)

    add_executable(foo2 foo2.c)
    target_link_libraries(foo2 HALCON::HALCONC)

    add_executable(fooxl2 fooxl2.c)
    target_link_libraries(fooxl2 HALCON::HALCONCXL)

    add_executable(foo3 foo3.cpp)
    target_link_libraries(foo3 HALCON::HALCONCPP)

    add_executable(fooxl3 fooxl3.cpp)
    target_link_libraries(fooxl3 HALCON::HALCONCPPXL)


#]=======================================================================]

option(HALCON_DEBUG "Print out internal HALCON CMake variables" OFF)
set(HALCON_FOUND FALSE)

# Make sure HALCON_ROOT and HALCON_ARCH variables are set
message(STATUS "Checking CMake variable HALCON_ROOT and env. variable HALCONROOT")
if("${HALCON_ROOT}" STREQUAL "")
    if ("$ENV{HALCONROOT}" STREQUAL "")
        message(FATAL_ERROR "Variable HALCON_ROOT was not set!")
    endif()
    message(WARNING "Variable HALCON_ROOT was not manually set, setting it to env variable HALCONROOT")
    set(HALCON_ROOT "$ENV{HALCONROOT}")
endif()

message(STATUS "Checking CMake variable HALCON_ARCH and env. variable HALCONARCH")
if("${HALCON_ARCH}" STREQUAL "")
    if ("$ENV{HALCONARCH}" STREQUAL "")
        message(FATAL_ERROR "Variable HALCON_ARCH was not set!")
    endif()
    message(WARNING "Variable HALCON_ARCH was not manually set, setting it to env variable HALCONARCH")
    set(HALCON_ARCH "$ENV{HALCONARCH}")
endif()

# Set platform specific library names
set(HALCON_LIBRARY_NAME      "halcon")
set(HALCONXL_LIBRARY_NAME    "halconxl")
set(HALCONC_LIBRARY_NAME     "halconc")
set(HALCONCXL_LIBRARY_NAME   "halconcxl")
set(HALCONCPP_LIBRARY_NAME   "halconcpp")
set(HALCONCPPXL_LIBRARY_NAME "halconcppxl")

# Set HALCON include variable
find_path(HALCON_INCLUDE_DIR
  NAMES
    Halcon.h
  HINTS
    ${HALCON_ROOT}
  PATH_SUFFIXES
    include
)

# Find and set HALCON library variable with custom macro
macro(find_halcon_library _var)
  find_library(${_var}
    NAMES
      ${ARGN}
    HINTS
      ${HALCON_ROOT}/lib/${HALCON_ARCH}
    PATH_SUFFIXES
      lib
      ${HALCON_ARCH}
  )
endmacro()

find_halcon_library(HALCON_LIBRARY      ${HALCON_LIBRARY_NAME})
find_halcon_library(HALCONXL_LIBRARY    ${HALCONXL_LIBRARY_NAME})
find_halcon_library(HALCONC_LIBRARY     ${HALCONC_LIBRARY_NAME})
find_halcon_library(HALCONCXL_LIBRARY   ${HALCONCXL_LIBRARY_NAME})
find_halcon_library(HALCONCPP_LIBRARY   ${HALCONCPP_LIBRARY_NAME})
find_halcon_library(HALCONCPPXL_LIBRARY ${HALCONCPPXL_LIBRARY_NAME})

# Set C# variables for Windows platform
if(WIN32)
  if("${HALCON_DOTNET}" STREQUAL "")
    message(WARNING "Variable HALCON_DOTNET was not manually set, setting it to default - dotnet35")
    set(HALCON_DOTNET "dotnet35")
  endif()
  set(MVTEC_HALCON_DOTNET_DIR "${HALCON_ROOT}/bin/${HALCON_DOTNET}")
  set(MVTEC_HALCON_DOTNET_LIBRARY "${MVTEC_HALCON_DOTNET_DIR}/halcondotnet.dll")
endif()

# Set variables as advanced (can be shown in GUI when Advanced is ON)
mark_as_advanced(
  HALCON_INCLUDE_DIR
  HALCON_LIBRARY      HALCON_LIBRARY_NAME
  HALCONXL_LIBRARY    HALCONXL_LIBRARY_NAME
  HALCONC_LIBRARY     HALCONC_LIBRARY_NAME
  HALCONCXL_LIBRARY   HALCONCPP_LIBRARY_NAME
  HALCONCPP_LIBRARY   HALCONCXL_LIBRARY_NAME
  HALCONCPPXL_LIBRARY HALCONCPPXL_LIBRARY_NAME
)

# Set <pckg>_FOUND flag if everything was found correctly
if(NOT "${HALCON_INCLUDE_DIR}" STREQUAL "")
  set(HALCON_FOUND TRUE)
endif()

# Create library targets with custom macro
macro(add_halcon_library TARGET)
  set(OPTIONS)
  set(ONE_VALUE LANG)
  set(MULTI_VALUE LINK_LIBS)
  cmake_parse_arguments(MY "${OPTIONS}" "${ONE_VALUE}" "${MULTI_VALUE}" ${ARGN})

  if(MY_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unknown arguments ${MY_UNPARSED_ARGUMENTS}")
  endif ()

  if(NOT TARGET ${TARGET})
    add_library(${TARGET} UNKNOWN IMPORTED)
    set_target_properties(${TARGET}
      PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES 
          "${HALCON_INCLUDE_DIR}"
    )
    set_target_properties(${TARGET}
      PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "${MY_LANG}"
        IMPORTED_LOCATION "${MY_LINK_LIBS}"
    )
  endif()
endmacro()

if(HALCON_FOUND)
  # halcon
  add_halcon_library(HALCON::HALCON
    LANG CXX
    LINK_LIBS ${HALCON_LIBRARY}
  )
  add_halcon_library(HALCON::HALCONXL
    LANG CXX
    LINK_LIBS ${HALCONXL_LIBRARY}
  )
  # halconc
  add_halcon_library(HALCON::HALCONC
    LANG C
    LINK_LIBS ${HALCONC_LIBRARY}
  )
  add_halcon_library(HALCON::HALCONCXL
    LANG C
    LINK_LIBS ${HALCONCXL_LIBRARY}
  )
  #halconcpp
  add_halcon_library(HALCON::HALCONCPP
    LANG CXX
    LINK_LIBS ${HALCONCPP_LIBRARY}
  )
  add_halcon_library(HALCON::HALCONCPPXL
    LANG CXX
    LINK_LIBS ${HALCONCPPXL_LIBRARY}
  )
endif()

# Print out debug information
if(HALCON_DEBUG)
  message(STATUS "===DEBUG: HALCON_FOUND = ${HALCON_FOUND}")

  message(STATUS "===DEBUG: HALCON_ROOT = ${HALCON_ROOT}")
  message(STATUS "===DEBUG: HALCON_ARCH = ${HALCON_ARCH}")

  message(STATUS "===DEBUG: HALCON_LIBRARY_NAME      = ${HALCON_LIBRARY_NAME}")
  message(STATUS "===DEBUG: HALCONXL_LIBRARY_NAME    = ${HALCONXL_LIBRARY_NAME}")
  message(STATUS "===DEBUG: HALCONC_LIBRARY_NAME     = ${HALCONC_LIBRARY_NAME}")
  message(STATUS "===DEBUG: HALCONCPP_LIBRARY_NAME   = ${HALCONCPP_LIBRARY_NAME}")
  message(STATUS "===DEBUG: HALCONCXL_LIBRARY_NAME   = ${HALCONCXL_LIBRARY_NAME}")
  message(STATUS "===DEBUG: HALCONCPPXL_LIBRARY_NAME = ${HALCONCPPXL_LIBRARY_NAME}")

  message(STATUS "===DEBUG: HALCON_LIBRARY      = ${HALCON_LIBRARY}")
  message(STATUS "===DEBUG: HALCONXL_LIBRARY    = ${HALCONXL_LIBRARY}")
  message(STATUS "===DEBUG: HALCONC_LIBRARY     = ${HALCONC_LIBRARY}")
  message(STATUS "===DEBUG: HALCONCXL_LIBRARY   = ${HALCONCXL_LIBRARY}")
  message(STATUS "===DEBUG: HALCONCPP_LIBRARY   = ${HALCONCPP_LIBRARY}")
  message(STATUS "===DEBUG: HALCONCPPXL_LIBRARY = ${HALCONCPPXL_LIBRARY}")

  message(STATUS "===DEBUG: HALCON_INCLUDE_DIR = ${HALCON_INCLUDE_DIR}")
endif()
