cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(CartoTCP LANGUAGES C CXX)

# TouchDesigner externals link the static CRT (/MT) on Windows. Set it for the
# whole addon before Avendish and the object library are created so every target
# shares one runtime. (CMP0091 is NEW via cmake_minimum_required.)
if(MSVC)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Where Avendish's avnd_make_touchdesigner looks for the TD headers
# (<PATH>/include/CPlusPlus_Common.h + POP_CPlusPlusBase.h). CI passes a full
# CustomOperatorSamples checkout via -DTOUCHDESIGNER_SDK_PATH; for local builds
# we default to the headers vendored under td-headers/ (which post-date POPs, so
# the build does not depend on an SDK that predates them). Left unset only when
# neither is available, in which case the POP back-end silently no-ops.
if(NOT TOUCHDESIGNER_SDK_PATH AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/td-headers/include/CPlusPlus_Common.h")
  set(TOUCHDESIGNER_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/td-headers"
      CACHE PATH "Directory with include/CPlusPlus_Common.h and include/POP_CPlusPlusBase.h")
endif()

# Use the Avendish the host already provides (e.g. ossia score); otherwise fetch it.
find_package(Avendish QUIET)
if(NOT Avendish_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    Avendish
    GIT_REPOSITORY "https://github.com/celtera/avendish"
    GIT_TAG 558744dcd18657163a1d6955ed9992188e00e8ad)
  FetchContent_Populate(Avendish)
  list(APPEND CMAKE_PREFIX_PATH "${avendish_SOURCE_DIR}")
  find_package(Avendish REQUIRED)
endif()

# What the object library and every back-end that links it need. Propagated via
# avnd_addon_object's DEPENDENCIES (linked PUBLIC to the object lib).
#  - Avendish::Avendish: the object's own translation unit includes <halp/*>, and
#    the internally-created object STATIC lib does not otherwise inherit Avendish's
#    include path (only the back-end targets do).
#  - cxx_std_23: std::span / std::is_nothrow_*_v used by net/*.hpp are C++20/23; the
#    object lib is built at the compiler default unless we require the standard here
#    (libstdc++ is lax, but macOS libc++ / MSVC reject the older standard).
#  - src include dir so "net/NetIO.hpp" resolves.
#  - ws2_32 on Windows: the TCP receiver uses the Winsock API (socket/bind/recv/
#    WSAStartup); without it the module fails to link (LNK2019 __imp_* externals).
add_library(cartotcp_deps INTERFACE)
target_link_libraries(cartotcp_deps INTERFACE Avendish::Avendish)
target_compile_features(cartotcp_deps INTERFACE cxx_std_23)
target_include_directories(cartotcp_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(WIN32)
  target_link_libraries(cartotcp_deps INTERFACE ws2_32)
endif()

avnd_addon_init(NAME CartoTCPAddon)

# BACKENDS touchdesigner:POP: receive an XYZ point cloud over TCP and expose it
# as a TouchDesigner POP point cloud. avnd_make_touchdesigner handles the POP
# entry-point generation, the compiler requirements (Clang/MSVC) and packaging.
# In a score build the back-end list is ignored (score introspects the object).
avnd_addon_object(
  BASE CartoTCPAddon
  C_NAME sat_cartotcp
  CLASS CartoTCP
  NAMESPACE sat
  BACKENDS touchdesigner:POP
  DEPENDENCIES cartotcp_deps
  MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/CartoTCP.hpp"
  SOURCES
    src/CartoTCP.hpp
    src/CartoTCP.cpp)

avnd_addon_finalize(NAME CartoTCPAddon UUID 8f2a3c1d-6b4e-4a9f-8c2d-1e5f7a9b0c3d VERSION 1)
