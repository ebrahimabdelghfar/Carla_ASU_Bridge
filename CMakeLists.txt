cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "asurt")

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_ENABLE_EXPORTS ON)
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-export-dynamic")
set(PARENT_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/libs CACHE PATH "Install prefix")
set(THIRD_PARTY_LIBS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/thirdparty)


project("${PROJECT_NAME}"
    VERSION 0.1
    DESCRIPTION "AUSRT Simulation Simulation"
    LANGUAGES C CXX)


# Set RPATH so installed shared libs can find each other at runtime
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH
    "${CMAKE_INSTALL_PREFIX}/libs"
)


##################### Build Simulation Libs ##########################
add_subdirectory(src/libs)
