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
find_package(LimeSuite REQUIRED)
find_package(LiquidDSP REQUIRED)

configure_file(config.hpp.in config.hpp @ONLY)
include_directories(${PROJECT_BINARY_DIR})

add_executable(wrsiggen)
add_executable(wrgpio)
add_executable(wrbeacon)

target_sources(wrsiggen PRIVATE
  wrsiggen.cpp
  config.hpp.in
)

target_include_directories(wrsiggen PRIVATE
  Boost::headers
)

target_link_libraries(wrsiggen
  PRIVATE
  Boost::system
  LimeSuite
)

target_sources(wrgpio PRIVATE
  wrgpio.cpp
  config.hpp.in
)

target_link_libraries(wrgpio
  PRIVATE
  LimeSuite
)

target_sources(wrbeacon PRIVATE
  frame.hpp
  frame.cpp
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
