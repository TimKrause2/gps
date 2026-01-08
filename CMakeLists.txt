#
# This is an example CMakeLists.txt file to compile a program against mathplot
#
cmake_minimum_required(VERSION 3.10)

# Give your project a name
project(gps)

# mathplot uses c++-20 language features
set(CMAKE_CXX_STANDARD 20)

# Set up CMAKE_CXX_FLAGS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-g -O3 -Wall -Wfatal-errors -DGL_SILENCE_DEPRECATION")
elseif (WIN32)
  set(CMAKE_CXX_FLAGS "-DNOMINMAX /EHsc /Zc:__cplusplus")
else()
  # This assumes a gcc compiler (or a gcc mimic like Clang)
  set(CMAKE_CXX_FLAGS "-g -O3 -Wall -Wextra -Wpedantic -pedantic-errors -Werror -Wfatal-errors -Wno-psabi -Wno-unknown-pragmas")
endif()

# Tell the program where the mathplot fonts are, to compile them into the binary
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMPLOT_FONTS_DIR=\"\\\"${PROJECT_SOURCE_DIR}/mathplot/fonts\\\"\"")

# Find the libraries which will be needed by mathplot
set(OpenGL_GL_PREFERENCE "GLVND") # Following `cmake --help-policy CMP0072`
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(Freetype REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Armadillo) # Optional
find_package(HDF5)      # Optional
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

pkg_search_module(FFTW3F_PKG REQUIRED IMPORTED_TARGET fftw3f)

# include files for graphics
set(MPLOT_INC_EXTRA ${ARMADILLO_INCLUDE_DIR} ${ARMADILLO_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR})
include_directories(${MPLOT_INC_EXTRA} ${OPENGL_INCLUDE_DIR} ${GLFW3_INCLUDE_DIR} ${FREETYPE_INCLUDE_DIRS})
# Assuming that you installed mathplot in-tree (i.e. 'next to' yourprog.cpp).
set(MPLOT_INCLUDE_PATH "${PROJECT_SOURCE_DIR}/mathplot" CACHE PATH "The path to mathplot")
include_directories(BEFORE ${MPLOT_INCLUDE_PATH})         # Allows mplot/Header.h to be found
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/maths)   # Allows sm/Header.h to be found

# If you want to add another directory to your include path, you can add an extra include_directories call:
# include_directories(BEFORE ${PROJECT_SOURCE_DIR}/my_includes)

#
# All the library finding is now done. It's time to compile the program.
#

add_executable(gps
  sensors.h sensors.cpp
  timer.h timer.cpp
  sensorshost.h sensorshost.cpp
)

target_sources(gps
  PRIVATE
    gps.cpp lfsr.cpp dco.cpp test_sig.cpp satellite.cpp
    search.cpp prns.cpp lnav.cpp triangulator.cpp
    constants.h dco.h gps.h lfsr.h test_sig.h
    satellite.h search.h prns.h lnav.h triangulate.h
    moving_avg.h ssiq.h queue.h
)

target_link_libraries(gps PRIVATE PkgConfig::FFTW3F_PKG Eigen3::Eigen OpenGL::GL Freetype::Freetype glfw)
# set(MPLOT_LIBS_EXTRA ${ARMADILLO_LIBRARY} ${ARMADILLO_LIBRARIES} ${HDF5_C_LIBRARIES})

#
# All done!
#

# This is just a useful stanza for debugging of cmake variables:
option(DEBUG_VARIABLES OFF)
if(DEBUG_VARIABLES)
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endif(DEBUG_VARIABLES)
