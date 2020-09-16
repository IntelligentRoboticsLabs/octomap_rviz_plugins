cmake_minimum_required(VERSION 3.5)

project(octomap_rviz_plugin)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

qt5_wrap_cpp(MOC_FILES
  include/octomap_rviz_plugin/occupancy_grid_display.hpp
  include/octomap_rviz_plugin/occupancy_map_display.hpp
  #OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_LEXICAL_CAST_INCLUDED 
)

set(SOURCE_FILES 
  src/occupancy_grid_display.cpp
  src/occupancy_map_display.cpp
  ${MOC_FILES} 
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt5::Widgets 
  octomap
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "OCTOMAP_RVIZ_PLUGIN_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  octomap
  octomap_msgs
  map_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
)

ament_export_include_directories(include)
ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rviz_ogre_vendor
  octomap
  octomap_msgs
  map_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include DESTINATION include)

ament_package()
