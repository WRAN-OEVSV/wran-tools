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

find_package(Boost     REQUIRED COMPONENTS system program_options)
find_package(LiquidDSP REQUIRED)

configure_file(config.hpp.in config.hpp @ONLY)
include_directories(${PROJECT_BINARY_DIR})

add_executable(wrbeacon-sim)

target_sources(wrbeacon-sim PRIVATE
  wranfrm.cpp
  wranfrm.hpp
  wrbeacon_sim.cpp
  config.hpp.in
)

target_link_libraries(wrbeacon-sim
  PRIVATE
  LiquidDSP
  Boost::program_options
)

add_executable(wrrx-sim)

target_sources(wrrx-sim PRIVATE
  wranfrm.cpp
  wranfrm.hpp
  wrrx_sim.cpp
  config.hpp.in
  AudioFile.h
)

target_link_libraries(wrrx-sim
  PRIVATE
  LiquidDSP
  Boost::system
  Boost::program_options
)

add_executable(hrsim)

target_sources(hrsim PRIVATE
    wranfrm.cpp
    wranfrm.hpp
    hrsim.cpp
    config.hpp.in
)

target_link_libraries(hrsim
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
)

install(TARGETS
  wrbeacon-sim
  wrrx-sim
  hrsim
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
    Boost::program_options
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
    Boost::program_options
  )

  add_executable(cwbeacon)

  target_sources(cwbeacon PRIVATE
    cwbeacon.cpp
    config.hpp.in
    morsecode.cpp
    morsecode.hpp
    keyer.cpp
    keyer.hpp
  )

  target_link_libraries(cwbeacon
    PRIVATE
    Boost::system
    LimeSuite
  )

  add_executable(wrbeacon)

  target_sources(wrbeacon PRIVATE
    wranfrm.cpp
    wranfrm.hpp
    wrbeacon.cpp
    config.hpp.in
    morsecode.cpp
    morsecode.hpp
    keyer.cpp
    keyer.hpp
  )

  target_link_libraries(wrbeacon
    PRIVATE
    LimeSuite
    LiquidDSP
    Boost::system
    Boost::program_options
  )

  add_executable(hrbeacon)

  target_sources(hrbeacon PRIVATE
    hamranfrm.cpp
    hamranfrm.hpp
    hrbeacon.cpp
    config.hpp.in
  )

  target_link_libraries(hrbeacon
    PRIVATE
    LimeSuite
    LiquidDSP
    Boost::program_options
  )

  add_executable(wrrx-lime)

  target_sources(wrrx-lime PRIVATE
    wranfrm.cpp
    wranfrm.hpp
    wrrx_lime.cpp
    config.hpp.in
  )

  target_link_libraries(wrrx-lime
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
    LimeSuite
  )

  install(TARGETS
    wrsiggen
    wrgpio
    wrbeacon
    wrrx-lime
    hrbeacon
    cwbeacon
  )

endif(LimeSuite_FOUND)

find_package(PkgConfig)
if (PkgConfig_FOUND)
#find_package(rtlsdr)
pkg_check_modules(rtlsdr IMPORTED_TARGET librtlsdr)
#if (rtlsdr_FOUND)
if (pkgcfg_lib_rtlsdr_rtlsdr)
  add_executable(wrrx-rtl)

  target_sources(wrrx-rtl PRIVATE
    wranfrm.cpp
    wranfrm.hpp
    wrrx_rtl.cpp
    config.hpp.in
  )

  target_link_libraries(wrrx-rtl
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
#    rtlsdr::rtlsdr
    PkgConfig::rtlsdr
  )

  install(TARGETS wrrx-rtl)

endif (pkgcfg_lib_rtlsdr_rtlsdr)
#endif(rtlsdr_FOUND)
endif(PkgConfig_FOUND)

if (PkgConfig_FOUND)
pkg_check_modules(HackRF IMPORTED_TARGET libhackrf)
if (pkgcfg_lib_HackRF_hackrf)

  add_executable(wrrx-hackrf)

  target_sources(wrrx-hackrf PRIVATE
    wranfrm.cpp
    wranfrm.hpp
    wrrx_hackrf.cpp
    config.hpp.in
    grcudp.hpp
    grcudp.cpp
  )

  target_link_libraries(wrrx-hackrf
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
    PkgConfig::HackRF
  )

install(TARGETS wrrx-hackrf)

endif(pkgcfg_lib_HackRF_hackrf)
endif(PkgConfig_FOUND)
include(CPack)
