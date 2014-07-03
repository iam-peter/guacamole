
find_path(OGDF_INCLUDE_DIR
          NAMES ogdf
          PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/include
          )

find_path(OGDF_LIBRARY_DIR
          NAMES libOGDF.so
          PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/lib
          )

find_library(OGDF_LIBRARY
            NAMES OGDF
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/lib
            PATH_SUFFIXES release
            )

mark_as_advanced(OGDF_INCLUDE_DIR OGDF_LIBRARY_DIR OGDF_LIBRARY)
