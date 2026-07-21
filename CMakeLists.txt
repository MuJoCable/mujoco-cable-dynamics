cmake_minimum_required(VERSION 3.16)
project(mujoco_cable_dynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MUJOCO_SOURCE_DIR "" CACHE PATH "Path to a MuJoCo source checkout")
set(MUJOCO_PYTHON_PACKAGE_DIR "" CACHE PATH "Path to a Python mujoco package directory")

find_package(mujoco CONFIG QUIET)

if(NOT mujoco_FOUND AND MUJOCO_SOURCE_DIR)
  if(EXISTS "${MUJOCO_SOURCE_DIR}/include/mujoco/mujoco.h")
    set(MUJOCO_EXTERNAL_INCLUDE_DIR "${MUJOCO_SOURCE_DIR}/include")
  else()
    message(FATAL_ERROR "MUJOCO_SOURCE_DIR does not contain include/mujoco/mujoco.h")
  endif()
endif()

if(NOT mujoco_FOUND AND NOT MUJOCO_EXTERNAL_INCLUDE_DIR AND MUJOCO_PYTHON_PACKAGE_DIR)
  if(EXISTS "${MUJOCO_PYTHON_PACKAGE_DIR}/include/mujoco/mujoco.h")
    set(MUJOCO_EXTERNAL_INCLUDE_DIR "${MUJOCO_PYTHON_PACKAGE_DIR}/include")
    file(GLOB MUJOCO_EXTERNAL_LIBRARY
         "${MUJOCO_PYTHON_PACKAGE_DIR}/libmujoco*${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
    list(GET MUJOCO_EXTERNAL_LIBRARY 0 MUJOCO_EXTERNAL_LIBRARY)
  else()
    message(FATAL_ERROR "MUJOCO_PYTHON_PACKAGE_DIR does not contain include/mujoco/mujoco.h")
  endif()
endif()

if(NOT mujoco_FOUND AND NOT MUJOCO_EXTERNAL_INCLUDE_DIR)
  message(STATUS "mujoco CONFIG package not found. Set MUJOCO_PYTHON_PACKAGE_DIR or MUJOCO_SOURCE_DIR.")
endif()

add_subdirectory(plugin)

include(GNUInstallDirs)
install(
  DIRECTORY cable_plugin_demos/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/mujoco-cable-dynamics/cable_plugin_demos
)
install(
  FILES LICENSE README.md README_zh.md VERSION
  DESTINATION ${CMAKE_INSTALL_DATADIR}/mujoco-cable-dynamics
)
