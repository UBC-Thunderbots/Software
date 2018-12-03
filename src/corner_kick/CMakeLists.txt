# CMake build script for Corner Kick

cmake_minimum_required(VERSION 2.8.3)
project(corner_kick)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

catkin_package()

###########
## Build ##
###########

# Build Corner Kick executable
add_custom_target(
  corner_kick-package
  ALL
  COMMENT Packaging up ${PROJECT_NAME}

  # Get dependencies
  COMMAND yarn

  # Only build if src has changed
  COMMAND ./scripts/compare_src.sh || yarn build

  BYPRODUCTS build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
