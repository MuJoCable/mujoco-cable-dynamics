# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_library(cable_unilateral SHARED)
target_sources(
  cable_unilateral
  PRIVATE register.cc
          native_route.h
          surface_route.cc
          surface_route.h
          unilateral_cable.cc
          unilateral_cable.h
)

target_include_directories(cable_unilateral PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(TARGET mujoco::mujoco)
  target_link_libraries(cable_unilateral PRIVATE mujoco::mujoco)
elseif(TARGET mujoco)
  target_link_libraries(cable_unilateral PRIVATE mujoco)
elseif(MUJOCO_EXTERNAL_INCLUDE_DIR)
  target_include_directories(cable_unilateral PRIVATE ${MUJOCO_EXTERNAL_INCLUDE_DIR})
  if(MUJOCO_EXTERNAL_LIBRARY)
    target_link_libraries(cable_unilateral PRIVATE ${MUJOCO_EXTERNAL_LIBRARY})
  elseif(APPLE)
    target_link_options(cable_unilateral PRIVATE -undefined dynamic_lookup)
  endif()
else()
  message(FATAL_ERROR "No MuJoCo target or include directory found.")
endif()

include(GNUInstallDirs)
install(
  TARGETS cable_unilateral
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
