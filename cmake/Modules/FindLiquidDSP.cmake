# SPDX-FileCopyrightText: 2022 Roland Schwarz <oe1rsa@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later

#[=======================================================================[.rst:
FindLiquidDSP
-------------

Finds the LiquidDSP library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported target, if found:

    LiquidDSP::LiquidDSP

Result Variables
^^^^^^^^^^^^^^^^

This module will define the following variables:

    LiquidDSP_FOUND
    LiquidDSP_VERSION
    LiquidDSP_INCLUDE_DIRS
    LiquidDSP_LIBRARIES

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

    LiquidDSP_INCLUDE_DIR
    LiquidDSP_LIBRARY

#]=======================================================================]


find_path(_INCLUDE_DIR
  NAMES liquid.h
  PATHS ${PC_LiquidDSP_DIRS}
        /usr/local/include
        /usr/include
  PATH_SUFFIXES
      liquid
  NO_CACHE
)

get_filename_component(LiquidDSP_INCLUDE_DIR ${_INCLUDE_DIR} DIRECTORY CACHE)

find_library(LiquidDSP_LIBRARY
  NAMES liquid
  PATHS /usr/local/lib
        /usr/lib
)

find_library(FFTW3_LIBRARY
  NAMES fftw3f
  PATHS /usr/local/lib
        /usr/lib
)

list(APPEND _LIBRARIES ${FFTW3_LIBRARY})

find_library(fec_LIBRARY
  NAMES fec
  PATHS /usr/local/lib
        /usr/lib
)
list(APPEND _LIBRARIES ${fec_LIBRARY})

# Read version from liquid.h include file
file(STRINGS ${LiquidDSP_INCLUDE_DIR}/liquid/liquid.h VERSION_LINE
     REGEX [[^#define *LIQUID_VERSION *".*"]])
string(REGEX MATCH [["(.*)"]] _ ${VERSION_LINE})
set(LiquidDSP_VERSION ${CMAKE_MATCH_1})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LiquidDSP
  FOUND_VAR LiquidDSP_FOUND
  REQUIRED_VARS
    LiquidDSP_LIBRARY
    LiquidDSP_INCLUDE_DIR
  VERSION_VAR LiquidDSP_VERSION
)

if(LiquidDSP_FOUND)
  set(LiquidDSP_LIBRARIES ${LiquidDSP_LIBRARY})
  set(LiquidDSP_INCLUDE_DIRS ${LiquidDSP_INCLUDE_DIR})
endif()

if(LiquidDSP_FOUND AND NOT TARGET LiquidDSP)
  add_library(LiquidDSP UNKNOWN IMPORTED)
  set_target_properties(LiquidDSP PROPERTIES
    IMPORTED_LOCATION "${LiquidDSP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LiquidDSP_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${_LIBRARIES}"
  )
endif()

unset(_LIBRARIES)

