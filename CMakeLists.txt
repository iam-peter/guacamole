################################################################
# Project GUACAMOLE
################################################################

PROJECT(GUACAMOLE CXX)

# version number
set(GUACAMOLE_MAJOR 0)
set(GUACAMOLE_MINOR 7)
set(GUACAMOLE_PATCH 1)
set(GUACAMOLE_VERSION ${GUACAMOLE_MAJOR}.${GUACAMOLE_MINOR}.${GUACAMOLE_PATCH})
set(GUACAMOLE_DESCRIPTION "GUACAMOLE - an astonishing virtual reality engine")
set(GUACAMOLE_HOMEPAGE "http://www.GUACAMOLE.org")
set(GUACAMOLE_EXENAME "GUACAMOLE")
set(GUACAMOLE_PACKAGENAME "GUACAMOLE")

# We require at least version 2.8.0
cmake_minimum_required(VERSION 2.8.0)

if (UNIX)
	find_package(PkgConfig)
endif(UNIX)

# Location where cmake first looks for modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

################################################################
# Configure and find libraries
################################################################
set(GLOBAL_EXT_DIR ${GUACAMOLE_SOURCE_DIR}/externals)

if (UNIX)
	pkg_check_modules(ASSIMP REQUIRED assimp)
	pkg_check_modules(JSON REQUIRED jsoncpp)
	pkg_check_modules(GL REQUIRED gl)
	pkg_check_modules(GLEW REQUIRED glew)
elseif (WIN32)
	include(FindAssimp)
endif (UNIX)

include(define_macros)
include(find_compiler)
include(FindSchism)
include(FindBoost)
include(FindZMQ)

if (UNIX)
	set (BULLET_ROOT "/opt/bullet/default" CACHE PATH "Set to your bullet install path.")
elseif (WIN32)
	set (BULLET_ROOT "externals/" CACHE PATH "Set to your bullet install path.")
endif (UNIX)

find_package(Bullet REQUIRED)
find_package(Json REQUIRED)

set(LIB_PATHS
    ${ASSIMP_LIBRARY_DIRS}
    ${JSON_LIBRARY_DIRS}
    ${GL_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
    ${BOOST_LIBRARY_DIRS}
    ${BULLET_LIBRARY_DIRS}
    ${SCHISM_LIBRARY_DIRS}
    ${ZMQ_LIBRARY_DIRS}
)

set(INCLUDE_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/build
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SCHISM_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}    
)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

IF (UNIX)
	set(CMAKE_CXX_FLAGS_RELEASE "-s -O4 --std=c++0x")
	set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall --std=c++0x")
ELSEIF(MSVC)
	set(CMAKE_CXX_FLAGS_RELEASE "-D NDEBUG -D _SECURE_SCL=0 -D _SCL_SECURE_NO_WARNINGS -D _CRT_SECURE_NO_DEPRECATE /MD /MP")
	set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG /MDd /Zi")
ENDIF(UNIX)

################################################################
# Create GuaRC
################################################################

add_subdirectory(guarc)

################################################################
# Generate RC data
################################################################

# list resource files
file(GLOB_RECURSE RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    resources/*
)

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated)
endif()

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated)
endif()


IF (UNIX)
    ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
      COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/guarc/guarc
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/R.inl
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
      DEPENDS ${RESOURCES} guarc
    )

    ADD_CUSTOM_TARGET(CompileResources ALL
       DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
    )

ELSEIF(MSVC)
    ADD_CUSTOM_TARGET(CompileResources ALL
       DEPENDS ${RESOURCES} guarc
    )

    SET (_GUARC_EXECUTABLE "")
    SET (_GUARC_EXECUTABLE "/guarc/$(Configuration)/guarc.exe")

    ADD_CUSTOM_COMMAND(TARGET CompileResources POST_BUILD
    	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}${_GUARC_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/R.inl
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
    )
ENDIF(UNIX)


################################################################
# Create libraries
################################################################

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(src)

################################################################
# Create plugins
################################################################

add_subdirectory(plugins)

################################################################
# Testing
################################################################

option (GUACAMOLE_TESTS "Set to enable testing.")

if (GUACAMOLE_TESTS)
  add_subdirectory(tests)
  enable_testing()
  add_test( NAME testGUA COMMAND runTests )
endif (GUACAMOLE_TESTS)

################################################################
# Summary
################################################################

message(STATUS "" )
message(STATUS "Summary:" )
message(STATUS " build type: ${CMAKE_BUILD_TYPE}" )
message(STATUS "" )
message(STATUS " schism:" )
message(STATUS "   library: ${SCHISM_LIBRARIES}" )
message(STATUS "   library path: ${SCHISM_LIBRARY_DIRS}" )
message(STATUS "   include: ${SCHISM_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " boost:" )
message(STATUS "   library: ${BOOST_LIBRARIES}" )
message(STATUS "   library path: ${BOOST_LIBRARY_DIRS}" )
message(STATUS "   include: ${BOOST_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " assimp:" )
message(STATUS "   library: ${ASSIMP_LIBRARIES}" )
message(STATUS "   library path: ${ASSIMP_LIBRARY_DIRS}" )
message(STATUS "   include: ${ASSIMP_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " json:" )
message(STATUS "   library: ${JSON_LIBRARIES}" )
message(STATUS "   library path: ${JSON_LIBRARY_DIRS}" )
message(STATUS "   include: ${JSON_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " cuda:" )
message(STATUS "   library: ${CUDA_LIBRARIES}" )
message(STATUS "   library path: ${CUDA_LIBRARY_DIRS}" )
message(STATUS "   include: ${CUDA_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " gl:" )
message(STATUS "   library: ${GL_LIBRARIES}" )
message(STATUS "   include: ${GL_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " glew:" )
message(STATUS "   library: ${GLEW_LIBRARIES}" )
message(STATUS "   include: ${GLEW_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " bullet:" )
message(STATUS "   library: ${BULLET_LIBRARIES}" )
message(STATUS "   library path: ${BULLET_LIBRARY_DIRS}" )
message(STATUS "   include: ${BULLET_INCLUDE_DIRS}" )
message(STATUS "" )
