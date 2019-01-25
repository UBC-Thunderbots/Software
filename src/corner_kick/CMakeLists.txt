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

  # Check current yarn version
  COMMAND yarn -v

  # Get dependencies
  # We enforce `yarn.lock` with flag --frozen-lockfile 
  # at this step to ensure our dependencies remain
  # consistent outside of development
  COMMAND yarn --silent --frozen-lockfile

  # Only build if src has changed
  COMMAND ./scripts/compare_src.sh || yarn build

  BYPRODUCTS build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
