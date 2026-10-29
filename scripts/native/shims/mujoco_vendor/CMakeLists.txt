cmake_minimum_required(VERSION 3.16)
project(mujoco_vendor VERSION 3.16.0)

find_package(ament_cmake REQUIRED)

# Upstream mujoco_vendor (pal-robotics) downloads DeepMind's prebuilt MuJoCo and
# hard-fails on Darwin ("macOS (Darwin) is not supported!"). conda-forge's libmujoco
# does ship osx-arm64, so this package re-exports that instead. It carries no MuJoCo of
# its own — it only makes conda's copy look like a vendor package to consumers.
find_package(mujoco REQUIRED)
ament_export_dependencies(mujoco)

# mujoco_ros2_control locates headers and libs by walking up from mujoco_vendor's cmake
# config dir and appending opt/mujoco_vendor:
#
#   MUJOCO_PREFIX = <mujoco_vendor_DIR>/../../..   -> this package's install prefix
#   MUJOCO_ROOT   = ${MUJOCO_PREFIX}/opt/mujoco_vendor
#
# It then probes ${MUJOCO_ROOT} for include/mujoco/mujoco.h, include/simulate/simulate.h
# and lib/libsimulate. Our prefix holds none of those — the conda env does. Symlinking
# opt/mujoco_vendor at the conda prefix satisfies every probe at once, including the
# `NOT EXISTS simulate.cc` test that selects the link-directly path upstream added for
# conda installs. That path is the only viable one here: conda ships libsimulate and
# simulate.h but no simulate.cc, so the compile-from-source branch has nothing to build.
#
# A symlink rather than a copy: the target is the live env this build already links
# against, so the two cannot drift, and an 11 MB library is not duplicated per build.
install(CODE "
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/opt\")
  if(NOT IS_SYMLINK \"\${CMAKE_INSTALL_PREFIX}/opt/mujoco_vendor\")
    file(CREATE_LINK \"$ENV{CONDA_PREFIX}\" \"\${CMAKE_INSTALL_PREFIX}/opt/mujoco_vendor\" SYMBOLIC)
  endif()
")

ament_package()
