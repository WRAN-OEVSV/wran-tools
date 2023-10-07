# SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.5)
cmake_policy(VERSION 3.5...3.27)

project(wran-tools
  VERSION 0.2.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make sure our local CMake Modules path comes first.
list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake/Modules)

find_package(CxxOpts   REQUIRED)
find_package(Boost     REQUIRED COMPONENTS system)
find_package(LiquidDSP REQUIRED)

configure_file(config.hpp.in config.hpp @ONLY)
include_directories(${PROJECT_BINARY_DIR})

add_executable(wrbeacon-sim)

target_sources(wrbeacon-sim PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  wrbeacon_sim.cpp
  config.hpp.in
)

target_link_libraries(wrbeacon-sim
  PRIVATE
  LiquidDSP
)

add_executable(wrrx-sim)

target_sources(wrrx-sim PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  wrrx_sim.cpp
  config.hpp.in
)

target_link_libraries(wrrx-sim
  PRIVATE
  LiquidDSP
  Boost::system
)

install(TARGETS
  wrbeacon-sim
  wrrx-sim
)

find_package(LimeSuite)
if (LimeSuite_FOUND)

add_executable(wrsiggen)

target_sources(wrsiggen PRIVATE
  wrsiggen.cpp
  config.hpp.in
)

target_link_libraries(wrsiggen
  PRIVATE
  Boost::system
  LimeSuite
)

add_executable(wrgpio)

target_sources(wrgpio PRIVATE
  wrgpio.cpp
  config.hpp.in
)

target_link_libraries(wrgpio
  PRIVATE
  LimeSuite
)

add_executable(wrbeacon)

target_sources(wrbeacon PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  wrbeacon.cpp
  config.hpp.in
)

target_link_libraries(wrbeacon
  PRIVATE
  LimeSuite
  LiquidDSP
)

install(TARGETS
  wrsiggen
  wrgpio
  wrbeacon
)

endif()


find_package(rtlsdr)
if (rtlsdr_FOUND)

add_executable(wrrx-rtl)

target_sources(wrrx-rtl PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  wrrx_rtl.cpp
  config.hpp.in
)

target_link_libraries(wrrx-rtl
  PRIVATE
  LiquidDSP
  Boost::system
  rtlsdr::rtlsdr
)

install(TARGETS wrrx-rtl)

endif()


