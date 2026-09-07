#include "Engine3dPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

#include <chrono>
#include <cstdlib>
#include <thread>

using mc_engine3d::EngineInterfaceCamera;

namespace mc_plugin
{

Engine3dPlugin::~Engine3dPlugin()
{
  render_running_ = false;
  if(render_thread_.isRunning())
  {
    render_thread_.wait();
  }
}

void Engine3dPlugin::ensureQtApp()
{
  if(QCoreApplication::instance() != nullptr)
  {
    return;
  }

  // No host application: default to the offscreen platform so the plugin still
  // works headless (ticker, mc_mujoco, real robot).
  if(qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM"))
  {
    qputenv("QT_QPA_PLATFORM", "offscreen");
  }

  static int argc = 1;
  static char arg0[] = "mc_engine3d";
  static char * argv[] = {arg0, nullptr};
  owned_app_ = std::make_unique<QGuiApplication>(argc, argv);
}

void Engine3dPlugin::checkCameraFrames(mc_control::MCGlobalController & controller) const
{
  for(const auto & cam : cameras_)
  {
    if(!controller.robot().hasFrame(cam->getFrame()))
    {
      mc_rtc::log::error_and_throw("Camera '{}' is attached to unknown robot frame '{}'", cam->name(), cam->getFrame());
    }
  }
}

void Engine3dPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  render_thread_.self = this;

  ensureQtApp();

  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  }
  node_ = rclcpp::Node::make_shared("mc_engine3d");
  it_ = std::make_shared<image_transport::ImageTransport>(node_);

  engine_ = std::make_unique<Engine3D>(Engine3D::DIRECT);

  if(config.has("MainCamera"))
  {
    // The engine already owns a main camera, configure it in place so it stays
    // part of the render loop and is destroyed with the engine.
    auto main_camera = std::make_shared<EngineInterfaceCamera>(engine_->getMainCamera(), config("MainCamera"), it_);
    cameras_.push_back(main_camera);
  }
  else
  {
    mc_rtc::log::error_and_throw("MainCamera has not been defined in the plugin configuration, see etc/Engine3d.yaml "
                                 "for an example");
  }

  if(config.has("Cameras"))
  {
    for(const auto & camera_config : config("Cameras"))
    {
      auto camera = std::make_shared<EngineInterfaceCamera>(camera_config, it_);
      cameras_.push_back(camera);
      engine_->addCamera(camera->camera().get());
    }
  }
  else
  {
    mc_rtc::log::warning("No 'Cameras' section defined for the Engine3d plugin, only the main camera will be streamed");
  }

  checkCameraFrames(controller);

  config("render_rate", render_rate_);

  if(config.has("mesh_model"))
  {
    model_path_ = static_cast<std::string>(config("mesh_model"));
  }
  else
  {
    mc_rtc::log::warning("Mesh model has not been defined in the plugin configuration");
  }

  // Engine3D::DIRECT only dispatches takePicture() on the thread that owns the
  // engine: hand the engine and every camera over to the render thread so the
  // control step never blocks on a render.
  engine_->moveToThread(&render_thread_);
  for(auto & cam : cameras_)
  {
    cam->camera()->moveToThread(&render_thread_);
  }

  render_running_ = true;
  render_thread_.start();

  mc_rtc::log::info("mc_engine3d initialized with {} camera(s)", cameras_.size());
}

void Engine3dPlugin::reset(mc_control::MCGlobalController & controller)
{
  // The controlled robot may have changed on a controller switch.
  checkCameraFrames(controller);
  mc_rtc::log::info("mc_engine3d plugin reset");
}

void Engine3dPlugin::before(mc_control::MCGlobalController &) {}

void Engine3dPlugin::after(mc_control::MCGlobalController & controller)
{
  // Publish the latest camera poses to the render thread.
  {
    std::lock_guard<std::mutex> lock(poses_mutex_);
    poses_.poses.resize(cameras_.size());
    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      poses_.poses[i] = controller.robot().frame(cameras_[i]->getFrame()).position();
    }
    poses_.stamp = node_->now();
    poses_.valid = true;
  }

  if(rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }
}

void Engine3dPlugin::renderLoop()
{
  // Engine3D (DIRECT) creates its GL context on and renders from the thread that
  // owns it; init() moved the engine and the cameras here before starting.
  engine_->initialize();

  if(!model_path_.empty())
  {
    mc_rtc::log::info("Loading model : {}", model_path_);
    engine_->openModel(QString::fromStdString(model_path_));
  }

  if(render_rate_ > 0)
  {
    mc_rtc::log::info("mc_engine3d render thread started, capped at {} Hz", render_rate_);
  }
  else
  {
    mc_rtc::log::info("mc_engine3d render thread started, uncapped");
  }

  const auto period = render_rate_ > 0 ? std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(1.0 / render_rate_))
                                       : std::chrono::steady_clock::duration::zero();

  size_t frames = 0;
  auto last_report = std::chrono::steady_clock::now();

  while(render_running_ && rclcpp::ok())
  {
    const auto tic = std::chrono::steady_clock::now();

    PoseSnapshot snap;
    {
      std::lock_guard<std::mutex> lock(poses_mutex_);
      snap = poses_;
    }

    if(snap.valid)
    {
      for(size_t i = 0; i < cameras_.size(); ++i)
      {
        cameras_[i]->transform(snap.poses[i]);
      }

      // Renders the scene once for every camera and caches each framebuffer.
      engine_->takePicture();

      for(auto & cam : cameras_)
      {
        cam->publish(snap.stamp);
      }
      ++frames;
    }

    const auto now = std::chrono::steady_clock::now();
    if(now - last_report >= std::chrono::seconds(5))
    {
      const double elapsed = std::chrono::duration<double>(now - last_report).count();
      mc_rtc::log::info("mc_engine3d rendering at {:.1f} Hz", frames / elapsed);
      frames = 0;
      last_report = now;
    }

    if(period != std::chrono::steady_clock::duration::zero())
    {
      std::this_thread::sleep_until(tic + period);
    }
    else if(!snap.valid)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration Engine3dPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = false;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("Engine3d", mc_plugin::Engine3dPlugin)
