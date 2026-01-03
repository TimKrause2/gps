cmake_minimum_required(VERSION 3.31)

project(gps)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

pkg_search_module(FFTW3F_PKG REQUIRED IMPORTED_TARGET fftw3f)

add_executable(gps)

target_sources(gps
  PRIVATE
    gps.cpp lfsr.cpp dco.cpp test_sig.cpp satellite.cpp
    search.cpp prns.cpp lnav.cpp triangulator.cpp
)

target_compile_options(gps PRIVATE -O3)

target_link_libraries(gps PRIVATE PkgConfig::FFTW3F_PKG Eigen3::Eigen)
