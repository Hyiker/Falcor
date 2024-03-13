# Add this directory for internal users.
include_directories(.)

# Find sources.
file(GLOB metis_sources *.c)

# Build libmetis.
add_library(metis ${METIS_LIBRARY_TYPE} ${GKlib_sources} ${metis_sources})

if(UNIX)
    target_link_libraries(metis m)
endif()

target_include_directories(metis
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    PRIVATE
    ${GKLIB_PATH}
)
