cmake_minimum_required(VERSION 3.14)
project(gps LANGUAGES CXX C)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)
include(FetchContent)

# Setup OpenGL
cmake_policy(SET CMP0072 NEW) # Pefer GLVND over legacy GL libraries
find_package(OpenGL REQUIRED)

# Setup GLFW
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "Generate installation target" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE) # For static library

FetchContent_Declare(
    glfw
    GIT_REPOSITORY "https://github.com/glfw/glfw"
    GIT_TAG "3.3.8"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

# Setup ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY "https://github.com/ocornut/imgui"
    GIT_TAG "v1.92.4"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)
set(IMGUI_SOURCE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
	${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
	${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_SOURCE})
target_include_directories(imgui PUBLIC "${imgui_SOURCE_DIR};${imgui_SOURCE_DIR}/backends/")
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# Setup ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY "https://github.com/epezent/implot.git"
    GIT_TAG "v0.17"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(implot)

set(IMPLOT_SOURCE_DIR ${implot_SOURCE_DIR})
set(IMPLOT_SOURCE
    ${IMPLOT_SOURCE_DIR}/implot.cpp
    ${IMPLOT_SOURCE_DIR}/implot_demo.cpp
    ${IMPLOT_SOURCE_DIR}/implot_items.cpp
)
add_library(implot STATIC ${IMPLOT_SOURCE})
target_include_directories(implot PUBLIC ${IMPLOT_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

# setup fftw3f and Eigen
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

pkg_search_module(FFTW3F_PKG REQUIRED IMPORTED_TARGET fftw3f)

# Add the executable
add_executable(gps
  sensors.h sensors.cpp
  timer.h timer.cpp
)

target_sources(gps
  PRIVATE
    gps.cpp lfsr.cpp dco.cpp test_sig.cpp satellite.cpp
    search.cpp prns.cpp lnav.cpp triangulator.cpp
    constants.h dco.h gps.h lfsr.h test_sig.h
    satellite.h search.h prns.h lnav.h triangulate.h
    moving_avg.h ssiq.h queue.h
)

target_link_libraries(gps PRIVATE PkgConfig::FFTW3F_PKG Eigen3::Eigen implot)

# Silence OpenGL deprecation warnings on macOS
if(APPLE)
    target_compile_definitions(example PRIVATE GL_SILENCE_DEPRECATION)
endif()
