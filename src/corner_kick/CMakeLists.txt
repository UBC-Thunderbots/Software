cmake_minimum_required(VERSION 2.8.3)
project(corner_kick)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

catkin_package()

###########
## Build ##
###########

## Package application
add_custom_target(
  corner_kick-package
  ALL
  COMMENT Packaging up ${PROJECT_NAME}

  # Make scripts executable
  COMMAND chmod +x ./scripts/compare_src.sh
  COMMAND chmod +x ./scripts/start_corner_kick.sh

  # Get dependencies
  COMMAND yarn

  # Only build if src has changed
  COMMAND ./scripts/compare_src.sh || yarn build

  BYPRODUCTS build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)