#pragma once

#include <Engine3D.h>
#include <mc_engine3d/engine_camera.h>
#include <mc_engine3d/engine_config.h>

#include <mc_control/GlobalPlugin.h>

#include <image_transport/image_transport.hpp>

#include <QGuiApplication>
#include <QThread>

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

using namespace MIS;

namespace mc_plugin
{

/**
 * @brief Renders a static scene from virtual cameras attached to robot frames
 *
 * The controller is stepped by whatever mc_rtc interface loads this plugin; the
 * plugin only reads the camera frame poses in after() and hands them to a
 * dedicated render thread that owns every Engine3D call, so a heavy render never
 * delays the control step.
 */
struct Engine3dPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~Engine3dPlugin() override;

private:
  /// Latest camera poses handed over from the control loop to the render loop
  struct PoseSnapshot
  {
    std::vector<sva::PTransformd> poses;
    rclcpp::Time stamp;
    bool valid = false;
  };

  /// Create a QGuiApplication if the host does not already provide one; Engine3D
  /// (DIRECT) needs one for its offscreen GL context
  void ensureQtApp();

  /// Check that every camera frame exists on the controlled robot
  void checkCameraFrames(mc_control::MCGlobalController & controller) const;

  /// Render + publish loop, runs on render_thread_
  void renderLoop();

  /// Owned only when the host had no QCoreApplication instance
  std::unique_ptr<QGuiApplication> owned_app_;

  std::unique_ptr<Engine3D> engine_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  std::vector<std::shared_ptr<mc_engine3d::EngineInterfaceCamera>> cameras_;

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
    Engine3dPlugin * self = nullptr;
    void run() override
    {
      self->renderLoop();
    }
  };
  RenderThread render_thread_;
};

} // namespace mc_plugin
