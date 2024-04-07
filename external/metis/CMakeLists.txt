cmake_minimum_required(VERSION 3.15)
project(METIS)

option(MESHOPT_BUILD_SHARED_LIBS "Build shared library" OFF)
option(METIS_INSTALL "Install METIS" OFF)
option(METIS_BUILD_TEST "Build test programs" OFF)

# Configure libmetis library.
if(METIS_SHARED)
    set(METIS_LIBRARY_TYPE SHARED)
else()
    set(METIS_LIBRARY_TYPE STATIC)
endif(METIS_SHARED)

set(GKLIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/GKlib)


include(${GKLIB_PATH}/GKlibSystem.cmake)

# Recursively look for CMakeLists.txt in subdirs.
add_subdirectory("libmetis")
# add_subdirectory("programs")
