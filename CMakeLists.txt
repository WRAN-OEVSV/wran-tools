# SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.5)
cmake_policy(VERSION 3.5...3.27)

project(wran-tools
  VERSION 0.4.0
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

add_executable(hrbeacon-sim)

target_sources(hrbeacon-sim PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  hrbeacon_sim.cpp
  config.hpp.in
)

target_link_libraries(hrbeacon-sim
  PRIVATE
  LiquidDSP
  Boost::program_options
)

add_executable(hrrx-sim)

target_sources(hrrx-sim PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  hrrx_sim.cpp
  config.hpp.in
  AudioFile.h
)

target_link_libraries(hrrx-sim
  PRIVATE
  LiquidDSP
  Boost::system
  Boost::program_options
)

add_executable(hrdebug)

target_sources(hrdebug PRIVATE
  hamranfrm.cpp
  hamranfrm.hpp
  hrdebug.cpp
  config.hpp.in
)

target_link_libraries(hrdebug
    PRIVATE
    LiquidDSP
)

install(TARGETS
  hrdebug
)

add_executable(hrbeacon-txrx-sim)

target_sources(hrbeacon-txrx-sim PRIVATE
    hamranfrm.cpp
    hamranfrm.hpp
    hrbeacon_txrx_sim.cpp
    config.hpp.in
)

target_link_libraries(hrbeacon-txrx-sim
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
)

install(TARGETS
  hrbeacon-sim
  hrrx-sim
  hrbeacon-txrx-sim
)

find_package(LimeSuite)
if (LimeSuite_FOUND)

  add_executable(siggen-lime)

  target_sources(siggen-lime PRIVATE
    siggen_lime.cpp
    config.hpp.in
  )

  target_link_libraries(siggen-lime
    PRIVATE
    Boost::system
    Boost::program_options
    LimeSuite
  )

  add_executable(gpio-lime)

  target_sources(gpio-lime PRIVATE
    gpio_lime.cpp
    config.hpp.in
  )

  target_link_libraries(gpio-lime
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

  add_executable(hrbeacon)

  target_sources(hrbeacon PRIVATE
    hamranfrm.cpp
    hamranfrm.hpp
    hrbeacon.cpp
    config.hpp.in
    morsecode.cpp
    morsecode.hpp
    keyer.cpp
    keyer.hpp
  )

  target_link_libraries(hrbeacon
    PRIVATE
    LimeSuite
    LiquidDSP
    Boost::system
    Boost::program_options
  )

#  add_executable(hrbeacon)

#  target_sources(hrbeacon PRIVATE
#    hamranfrm.cpp
#    hamranfrm.hpp
#    hrbeacon.cpp
#    config.hpp.in
#  )

#  target_link_libraries(hrbeacon
#    PRIVATE
#    LimeSuite
#    LiquidDSP
#    Boost::program_options
#  )

  add_executable(hrrx-lime)

  target_sources(hrrx-lime PRIVATE
    hamranfrm.cpp
    hamranfrm.hpp
    hrrx_lime.cpp
    config.hpp.in
  )

  target_link_libraries(hrrx-lime
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
    LimeSuite
  )

  install(TARGETS
    siggen-lime
    gpio-lime
    hrbeacon
    hrrx-lime
    cwbeacon
  )

endif(LimeSuite_FOUND)

find_package(PkgConfig)
if (PkgConfig_FOUND)
#find_package(rtlsdr)
pkg_check_modules(rtlsdr IMPORTED_TARGET librtlsdr)
#if (rtlsdr_FOUND)
if (pkgcfg_lib_rtlsdr_rtlsdr)
  add_executable(hrrx-rtl)

  target_sources(hrrx-rtl PRIVATE
    hamranfrm.cpp
    hamranfrm.hpp
    hrrx_rtl.cpp
    config.hpp.in
  )

  target_link_libraries(hrrx-rtl
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
#    rtlsdr::rtlsdr
    PkgConfig::rtlsdr
  )

  install(TARGETS hrrx-rtl)

endif (pkgcfg_lib_rtlsdr_rtlsdr)
#endif(rtlsdr_FOUND)
endif(PkgConfig_FOUND)

if (PkgConfig_FOUND)
pkg_check_modules(HackRF IMPORTED_TARGET libhackrf)
if (pkgcfg_lib_HackRF_hackrf)

  add_executable(hrrx-hackrf)

  target_sources(hrrx-hackrf PRIVATE
    hamranfrm.cpp
    hamranfrm.hpp
    hrrx_hackrf.cpp
    config.hpp.in
    grcudp.hpp
    grcudp.cpp
  )

  target_link_libraries(hrrx-hackrf
    PRIVATE
    LiquidDSP
    Boost::system
    Boost::program_options
    PkgConfig::HackRF
  )

install(TARGETS hrrx-hackrf)

endif(pkgcfg_lib_HackRF_hackrf)
endif(PkgConfig_FOUND)
include(CPack)
