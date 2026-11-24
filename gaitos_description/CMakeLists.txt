# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robots launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "meshes" EXCLUDE PATTERN "assets" EXCLUDE
  PATTERN "meshes_dex3" EXCLUDE)   # the assets come from FetchContent
install(PROGRAMS scripts/make_gazebo_urdf.py scripts/make_mjcf.py scripts/dae_to_obj.py
  DESTINATION lib/${PROJECT_NAME})

# Meshes and MuJoCo assets of the Unitree robots (BSD-3-Clause, Unitree Robotics), not versioned
# with gaitos: downloaded at configure time (archives of pinned commits) and installed next to
# the URDF / MJCF of each robot. Offline or with local copies:
#   -DFETCHCONTENT_SOURCE_DIR_UNITREE_ROS=<unitree_ros> -DFETCHCONTENT_SOURCE_DIR_UNITREE_MUJOCO=<unitree_mujoco>
# Without them (no MuJoCo models, no meshes in RViz / Gazebo, also for the NAO below):
# -DGAITOS_FETCH_ASSETS=OFF
option(GAITOS_FETCH_ASSETS "Download the meshes and MuJoCo assets of the robots" ON)
if(GAITOS_FETCH_ASSETS)
  include(FetchContent)
  set(UNITREE_ROS_TAG 157ff950351e5f535e03c5d85a16b7d594b3dbcc CACHE STRING "unitree_ros commit")
  set(UNITREE_MUJOCO_TAG 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d CACHE STRING "unitree_mujoco commit")
  FetchContent_Declare(unitree_ros
    URL https://github.com/unitreerobotics/unitree_ros/archive/${UNITREE_ROS_TAG}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no_cmake)   # only download
  FetchContent_Declare(unitree_mujoco
    URL https://github.com/unitreerobotics/unitree_mujoco/archive/${UNITREE_MUJOCO_TAG}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no_cmake)
  FetchContent_MakeAvailable(unitree_ros unitree_mujoco)

  set(UROS ${unitree_ros_SOURCE_DIR}/robots)
  set(UMJ ${unitree_mujoco_SOURCE_DIR}/unitree_robots)
  set(ROBOTS share/${PROJECT_NAME}/robots)
  # <from> <to>: folder contents
  set(ASSETS
    ${UROS}/go2_description/dae   ${ROBOTS}/go2_description/meshes
    ${UMJ}/go2/assets             ${ROBOTS}/go2_description/mjcf/assets
    ${UROS}/r1_description/meshes ${ROBOTS}/r1_description/meshes
    ${UMJ}/r1/meshes              ${ROBOTS}/r1_description/mjcf/meshes)
  while(ASSETS)
    list(POP_FRONT ASSETS from to)
    if(NOT IS_DIRECTORY ${from})
      message(FATAL_ERROR "Not found in the Unitree repositories: ${from}")
    endif()
    install(DIRECTORY ${from}/ DESTINATION ${to})
  endwhile()
  # R1 with Dex3-1 hands (mjcf/r1_dex3.xml, urdf/r1_dex3.urdf): its own meshes (stock colours,
  # clipped wrists) plus the Unitree R1 meshes above and the G1 hand meshes
  install(DIRECTORY robots/r1_description/mjcf/meshes_dex3/ DESTINATION ${ROBOTS}/r1_description/mjcf/meshes
    PATTERN "LICENSE.md" EXCLUDE)
  install(DIRECTORY ${UROS}/g1_description/meshes/ DESTINATION ${ROBOTS}/r1_description/mjcf/meshes
    FILES_MATCHING PATTERN "*_hand_*_link.STL")

  install(FILES ${unitree_ros_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses
    RENAME unitree_ros.LICENSE)
  install(FILES ${unitree_mujoco_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses
    RENAME unitree_mujoco.LICENSE)
endif()

# NAO meshes: the Gazebo Fuel model "NAO with Ignition position controller" (OpenRobotics,
# from osrf/robocup3ds), Creative Commons Attribution 4.0. The .dae (and their texture) go next
# to the URDF for RViz and Gazebo; dae_to_obj.py converts them to OBJ for the MuJoCo model
# (mjcf/nao.xml, made by make_mjcf.py). Offline: -DFETCHCONTENT_SOURCE_DIR_NAO_FUEL=<unzipped model>
if(GAITOS_FETCH_ASSETS)
  set(NAO_FUEL_VERSION 2 CACHE STRING "Version of the NAO model in Gazebo Fuel")
  FetchContent_Declare(nao_fuel
    URL "https://fuel.gazebosim.org/1.0/OpenRobotics/models/NAO%20with%20Ignition%20position%20controller/${NAO_FUEL_VERSION}/NAO%20with%20Ignition%20position%20controller.zip"
    URL_HASH SHA256=f44767c0fc7bc614002c2c026089328027593679b4e9bff270d3e37318984c5d
    DOWNLOAD_NAME nao_fuel.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no_cmake)
  FetchContent_MakeAvailable(nao_fuel)
  if(NOT IS_DIRECTORY ${nao_fuel_SOURCE_DIR}/meshes/V40)
    message(FATAL_ERROR "No meshes/V40 in the NAO model of Fuel: ${nao_fuel_SOURCE_DIR}")
  endif()

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(NAO_OBJ ${CMAKE_CURRENT_BINARY_DIR}/nao_obj)
  file(GLOB NAO_DAE ${nao_fuel_SOURCE_DIR}/meshes/V40/*.dae)
  add_custom_command(OUTPUT ${NAO_OBJ}/Torso.obj
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/dae_to_obj.py
      ${nao_fuel_SOURCE_DIR}/meshes/V40 ${NAO_OBJ}
    DEPENDS ${NAO_DAE} scripts/dae_to_obj.py
    COMMENT "NAO meshes: .dae -> .obj for MuJoCo")
  add_custom_target(nao_obj ALL DEPENDS ${NAO_OBJ}/Torso.obj)

  set(NAO ${ROBOTS}/nao_description)
  install(DIRECTORY ${nao_fuel_SOURCE_DIR}/meshes/V40/ DESTINATION ${NAO}/meshes/V40)
  install(DIRECTORY ${nao_fuel_SOURCE_DIR}/materials/texture/ DESTINATION ${NAO}/materials/texture)
  install(DIRECTORY ${NAO_OBJ}/ DESTINATION ${NAO}/mjcf/meshes)
  install(FILES robots/nao_description/MESHES_LICENSE.md DESTINATION ${NAO}/meshes
    RENAME LICENSE.md)
endif()

# The URDFs refer to their meshes as package://<robot>_description/...: each robot folder is
# also a package of the ament index (a link share/<robot>_description -> robots/<robot>_description),
# so RViz (resource_retriever) resolves them like Gazebo does through GZ_SIM_RESOURCE_PATH.
# A package of that name already in the environment (e.g. r1_description of unitree_r1, with
# the same meshes) is left alone.
foreach(robot go2 r1 nao)
  set(pkg ${robot}_description)
  ament_index_has_resource(other packages ${pkg})
  if(other AND NOT other MATCHES "${PROJECT_NAME}")
    message(STATUS "${pkg}: the package of ${other} provides the meshes")
    continue()
  endif()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ament_index/${pkg} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ament_index/${pkg}
    DESTINATION share/ament_index/resource_index/packages)
  install(CODE "file(CREATE_LINK ${PROJECT_NAME}/robots/${pkg}
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${pkg} SYMBOLIC)")
endforeach()

ament_package()
