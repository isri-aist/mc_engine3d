#pragma once

#include <Engine3D.h>
#include <mc_engine3d/engine_camera.h>
#include <mc_engine3d/engine_config.h>

#include <mc_control/mc_global_controller.h>

#include <image_transport/image_transport.hpp>

#include <QThread>

#include <atomic>
#include <mutex>
#include <vector>

using namespace MIS;

namespace mc_engine3d
{

class EngineInterface
{
public:
  EngineInterface(int argc, char ** argv);

  ~EngineInterface();

  void init(const std::string & config = "");

  /**
   * @brief Run the control loop
   *
   * The controller is stepped on this thread at the controller timestep while
   * the scene is rendered and the camera streams published from a separate
   * thread, so a heavy render never delays the control step.
   */
  void run();

  inline bool running()
  {
    return gc_->running;
  }

private:
  /// Latest camera poses handed over from the control loop to the render loop
  struct PoseSnapshot
  {
    std::vector<sva::PTransformd> poses;
    rclcpp::Time stamp;
    bool valid = false;
  };

  /// Render + publish loop, runs on render_thread_
  void renderLoop();

  std::unique_ptr<mc_control::MCGlobalController> gc_;
  Engine3D engine_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  std::vector<std::shared_ptr<EngineInterfaceCamera>> cameras_;

  std::string model_path_;
  /// Render/publish rate cap in Hz, <= 0 means render as fast as possible
  double render_rate_ = 30.0;

  std::mutex poses_mutex_;
  PoseSnapshot poses_;

  std::atomic<bool> render_running_{false};

  /// QThread so the engine can be given this thread's affinity (Engine3D::DIRECT
  /// dispatches takePicture() on its own thread only)
  struct RenderThread : QThread
  {
    EngineInterface * self = nullptr;
    void run() override
    {
      self->renderLoop();
    }
  };
  RenderThread render_thread_;
};
} // namespace mc_engine3d
