cmake_minimum_required(VERSION 3.10)
project(Vis VERSION 1.2.0)

set(CMAKE_BUILD_TYPE "Release")
# Use C++17
set(DEFAULT_CXX_STANDARD ${CMAKE_CXX_STANDARD})
if (NOT DEFAULT_CXX_STANDARD)
  set(DEFAULT_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD ${DEFAULT_CXX_STANDARD} CACHE STRING "C++ standard (minimum 17)" FORCE)

# Make the lib all -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

# Set OpenGL_GL_PREFERENCE to LEGACY to silence warnings on cmake 3.11+
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

if(WIN32)
  find_package(OpenSceneGraph 3.6 REQUIRED COMPONENTS osgViewer osgGA osgText osgDB osgUtil osgFX)
  set(OSG_INCLUDE_DIR ${OPENSCENEGRAPH_INCLUDE_DIRS})
  set(OSG_LIBS ${OPENSCENEGRAPH_LIBRARIES})
else(WIN32)
  # install OpenSceneGraph with `sudo apt install libopenscenegraph-dev`
  find_package(osg 3.6 REQUIRED COMPONENTS osgViewer osgGA osgText osgDB osgUtil osgFX)
  set(OSG_LIBS osg osgViewer osgGA osgText osgDB osgUtil osgFX)
endif(WIN32)

add_subdirectory(Externals)

set(VIS_INC ${CMAKE_CURRENT_SOURCE_DIR}/Include)

set(VIS_CPPS 
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/Vis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/Manipulator/TouchballManipulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/Intersector/LineIntersector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/Intersector/PointIntersector.cpp
  )
set(VIS_DEPS Threads::Threads ${OSG_LIBS} fmt::fmt spdlog::spdlog libgizmo::libgizmo OpenGL::GL)

# Make libs and executables output into same place.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Bin)

add_library(Vis ${VIS_CPPS})
set_target_properties(Vis PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
target_include_directories(Vis PUBLIC ${VIS_INC} ${OSG_INCLUDE_DIR})
target_link_libraries(Vis PUBLIC ${VIS_DEPS})

add_library(Vis::Vis ALIAS Vis)

set(CMAKE_INSTALL_PREFIX "/opt/RVBUST")

file(MAKE_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

add_subdirectory(Examples)

pybind11_add_module(PyVis
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/Python/PyVis.cpp
  )
target_link_libraries(PyVis PRIVATE Vis)
set_target_properties(PyVis PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(PyVis PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(PyVis
  PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Python/RVBUST/Vis/"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Python/RVBUST/Vis/"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
  )


# import objects into Vision namespace 
FILE(WRITE "${CMAKE_BINARY_DIR}/Python/RVBUST/Vis/__init__.py"
  "from .PyVis import *\n"
  )  

# Install
install(DIRECTORY Include
  DESTINATION "${CMAKE_INSTALL_PREFIX}/Vis"
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  DESTINATION "${CMAKE_INSTALL_PREFIX}/Vis"
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  DESTINATION "${CMAKE_INSTALL_PREFIX}/Vis"
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY ${CMAKE_BINARY_DIR}/Python
  DESTINATION "${CMAKE_INSTALL_PREFIX}/Vis"
  USE_SOURCE_PERMISSIONS
)

set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "RVBUST Inc.")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION}-${CMAKE_BUILD_TYPE})
set(CPACK_PACKAGE_CONTACT "example@qq.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "example@qq.com")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libopenscenegraph-3.4-dev")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/postinst")
include(InstallRequiredSystemLibraries)
include(CPack)