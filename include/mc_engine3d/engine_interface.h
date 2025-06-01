#pragma once

#include <Engine3D.h>
#include <mc_engine3d/engine_camera.h>
#include <mc_engine3d/engine_config.h>

#include <mc_control/mc_global_controller.h>

#include <image_transport/image_transport.hpp>

using namespace MIS;

namespace mc_engine3d
{

class EngineInterface
{
public:
  EngineInterface(int argc, char ** argv);

  ~EngineInterface();

  void init(const std::string & config = "");

  void run();

  inline bool running()
  {
    return gc_->running;
  }

private:
  std::unique_ptr<mc_control::MCGlobalController> gc_;
  Engine3D engine_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  std::vector<std::shared_ptr<EngineInterfaceCamera>> cameras_;
};
} // namespace mc_engine3d
