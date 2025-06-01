cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_engine3d)
set(PROJECT_DESCRIPTION "Engine3D / mc_rtc interface")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)

find_package(OpenCV REQUIRED COMPONENTS imgcodecs)
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(Engine3D REQUIRED)
find_package(mc_rtc REQUIRED)

include_directories(include)

add_subdirectory(src)


