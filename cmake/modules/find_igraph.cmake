
find_path(IGRAPH_INCLUDE_DIR
          NAMES igraph
          PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/include
          )

find_path(IGRAPH_LIBRARY_DIR
          NAMES libIGRAPH.so
          PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/lib
          )

find_library(IGRAPH_LIBRARY
            NAMES IGRAPH
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/lib
            PATH_SUFFIXES release
            )

mark_as_advanced(IGRAPH_INCLUDE_DIR IGRAPH_LIBRARY_DIR IGRAPH_LIBRARY)
