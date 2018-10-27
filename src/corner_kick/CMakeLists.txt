cmake_minimum_required(VERSION 2.8.3)
project(corner_kick)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

catkin_package()

###########
## Build ##
###########

## Clean up pre-installed and pre-built packages from the src directory
add_custom_target(
  clean_src_npm
  COMMENT Removing ${PROJECT_NAME}'s modules and builds
  COMMAND rm -rf node_modules build dist
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

## Copy files over to devel for build and installation
add_custom_target(
  copy_src_to_devel
  DEPENDS clean_src_npm
  COMMENT Copying ${PROJECT_NAME}'s src files to devel
  COMMAND cp -rf ${PROJECT_SOURCE_DIR} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  COMMAND rm -rf ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/launch/
)

## Install NPM dependencies
add_custom_target(
  corner_kick-dependencies
  DEPENDS copy_src_to_devel
  COMMENT Installing ${PROJECT_NAME}'s dependencies
  COMMAND
  yarn install
    --cwd ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
  BYPRODUCTS node_modules
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

## Package application
add_custom_target(
  corner_kick-package
  ALL
  DEPENDS corner_kick-dependencies
  COMMENT Packaging up ${PROJECT_NAME}
  COMMAND yarn build
  COMMAND yarn dist
  BYPRODUCTS dist
  BYPRODUCTS build
  WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)