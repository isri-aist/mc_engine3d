cmake_minimum_required(VERSION 3.14)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_engine3d)
set(PROJECT_DESCRIPTION "Engine3D / mc_rtc plugin")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(OpenCV REQUIRED COMPONENTS imgcodecs)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(Engine3D REQUIRED) # also brings in Qt (Core/Gui/Widgets/OpenGL)

# Configure and install the plugin configuration
configure_file(etc/Engine3d.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/Engine3d.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/Engine3d.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/Engine3dPlugin.cpp src/engine_camera.cpp)

set(plugin_HDR src/Engine3dPlugin.h include/mc_engine3d/engine_camera.h
               include/mc_engine3d/engine_config.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(Engine3d "${plugin_SRC}" "${plugin_HDR}")

target_include_directories(Engine3d PRIVATE include ${Engine3D_INCLUDE_DIRS})

target_link_libraries(
  Engine3d PUBLIC ${Engine3D_LIBRARIES} cv_bridge::cv_bridge
                  image_transport::image_transport rclcpp::rclcpp)
