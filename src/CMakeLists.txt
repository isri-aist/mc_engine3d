# Create Engine3d library 
add_library(engine3d_interface SHARED
    engine_interface.cpp
    engine_camera.cpp
)

target_include_directories(engine3d_interface PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include
    $<INSTALL_INTERFACE:include>
    ${Engine3D_INCLUDE_DIRS}
)

target_link_libraries(engine3d_interface PUBLIC 
    mc_rtc::mc_control 
    ${Engine3D_LIBRARIES}
    cv_bridge::cv_bridge
    image_transport::image_transport
    rclcpp::rclcpp
)

# Create executable 
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
  engine3d_interface
  Boost::program_options
)

# Install executables
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
