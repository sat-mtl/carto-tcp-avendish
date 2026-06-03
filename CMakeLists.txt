cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(CartoTCP LANGUAGES C CXX)

# Directory holding the TouchDesigner POP headers. We vendor both
# CPlusPlus_Common.h and POP_CPlusPlusBase.h here so the build does not depend
# on a TOUCHDESIGNER_SDK_PATH that predates POPs — they are added via
# target_include_directories below.
set(TD_POP_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/td-headers"
    CACHE PATH "Directory with CPlusPlus_Common.h and POP_CPlusPlusBase.h")

include(FetchContent)

FetchContent_Declare(
  avendish
  GIT_REPOSITORY "https://github.com/celtera/avendish"
  GIT_TAG  11a6ae1b404b24bfec0e2841751eee3bc21cb60d
  GIT_PROGRESS true
)
FetchContent_Populate(avendish)
set(CMAKE_PREFIX_PATH "${avendish_SOURCE_DIR};${CMAKE_PREFIX_PATH}")
# FetchContent exposes the checkout as ${avendish_SOURCE_DIR}; the rest of this
# file refers to it as AVENDISH_PATH (the variable the old local-copy build used).
set(AVENDISH_PATH "${avendish_SOURCE_DIR}")
find_package(Avendish REQUIRED)
find_package(Boost REQUIRED)

# --- POP binding instantiation ----------------------------------------------
# Generate the TD POP entry-point translation unit from Avendish's prototype.
set(AVND_MAIN_FILE  "${CMAKE_CURRENT_SOURCE_DIR}/src/CartoTCP.hpp")
set(AVND_MAIN_CLASS "sat::CartoTCP")
set(AVND_C_NAME     "sat_cartotcp")
set(AVND_OPTYPE     "")

configure_file(
  "${AVENDISH_PATH}/include/avnd/binding/touchdesigner/POP.prototype.cpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/CartoTCP_POP_touchdesigner.cpp"
  @ONLY
  NEWLINE_STYLE LF
)

# --- Plugin module ----------------------------------------------------------
add_library(CartoTCP_pop MODULE
  "${AVND_MAIN_FILE}"
  "${CMAKE_CURRENT_BINARY_DIR}/CartoTCP_POP_touchdesigner.cpp"
)

set_target_properties(CartoTCP_pop PROPERTIES
  PREFIX ""                       # no "lib" prefix; TD loads CartoTCP_pop.{dll,dylib}
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  LIBRARY_OUTPUT_DIRECTORY td
  RUNTIME_OUTPUT_DIRECTORY td
)

# Both the vendored TD POP headers and the Avendish headers, as SYSTEM includes.
target_include_directories(CartoTCP_pop SYSTEM PRIVATE
  "${TD_POP_HEADERS}"
  "${AVENDISH_PATH}/include"
)
# Our own sources (so net/NetIO.hpp etc. resolve relative to src/).
target_include_directories(CartoTCP_pop PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(CartoTCP_pop PRIVATE
  Avendish::Avendish
  Boost::boost
)

if(APPLE)
  set_target_properties(CartoTCP_pop PROPERTIES
    SUFFIX ".dylib"
    BUNDLE FALSE
    MACOSX_BUNDLE FALSE
  )
elseif(WIN32)
  set_target_properties(CartoTCP_pop PROPERTIES SUFFIX ".dll")
endif()

# The TouchDesigner SDK headers use __cdecl and the MSVC ABI: build with Clang
# on Linux/macOS, or MSVC on Windows. GCC does not parse __cdecl on x86-64, and
# MinGW is ABI-incompatible with TD. Refuse early with guidance (upstream's
# avnd_make_touchdesigner does the same).
if(MINGW OR CYGWIN OR MSYS OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  message(FATAL_ERROR
    "CartoTCP (TouchDesigner POP) requires Clang (Linux/macOS) or MSVC (Windows); "
    "GCC/MinGW cannot parse the TD SDK headers. Reconfigure a clean build dir with:\n"
    "  cmake -B build -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(CartoTCP_pop PRIVATE -fvisibility=hidden)
endif()

message(STATUS "Configured TouchDesigner POP: CartoTCP_pop -> td/CartoTCP_pop")
