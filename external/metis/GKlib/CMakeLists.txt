cmake_minimum_required(VERSION 2.8)
project(GKlib)

get_filename_component(abs "." ABSOLUTE)
# set(GKLIB_PATH ${abs})
# unset(abs)
include(GKlibSystem.cmake)

add_library(GKlib STATIC ${GKlib_sources})
target_include_directories(GKlib PUBLIC ".")

if(UNIX)
    target_link_libraries(GKlib m)
endif(UNIX)

if(METIS_BUILD_TEST)
    include_directories("test")
    add_subdirectory("test")
endif()

if(METIS_INSTALL)
    install(TARGETS GKlib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
    install(FILES ${GKlib_includes} DESTINATION include)
endif(METIS_INSTALL)
