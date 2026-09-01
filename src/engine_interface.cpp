#include <mc_engine3d/engine_interface.h>

#include <chrono>
#include <thread>

using namespace mc_engine3d;

EngineInterface::EngineInterface(int argc, char ** argv) : engine_(Engine3D::DIRECT)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  node_ = rclcpp::Node::make_shared("mc_engine3d");
  it_ = std::make_shared<image_transport::ImageTransport>(node_);
  render_thread_.self = this;
}

EngineInterface::~EngineInterface()
{
  render_running_ = false;
  if(render_thread_.isRunning())
  {
    render_thread_.wait();
  }
}

void EngineInterface::init(const std::string & config_file)
{
  mc_control::MCGlobalController::GlobalConfiguration gconfig(config_file);

  if(!gconfig.config.has("Engine3d"))
  {
    mc_rtc::log::error_and_throw("No Engine3d section in the configuration, see etc/mc_engine3d.yaml for an example");
  }

  gc_ = std::make_unique<mc_control::MCGlobalController>(gconfig);
  gc_->init();
  gc_->running = true;

  auto engine3DConfig = gconfig.config("Engine3d");

  if(engine3DConfig.has("MainCamera"))
  {
    // The engine already owns a main camera, configure it in place so it stays
    // part of the render loop and is destroyed with the engine.
    auto main_camera =
        std::make_shared<EngineInterfaceCamera>(engine_.getMainCamera(), engine3DConfig("MainCamera"), it_);
    cameras_.push_back(main_camera);
  }
  else
  {
    mc_rtc::log::error_and_throw("MainCamera has not been defined in the configuration file");
  }

  if(engine3DConfig.has("Cameras"))
  {
    for(const auto & camera_config : engine3DConfig("Cameras"))
    {
      auto camera = std::make_shared<EngineInterfaceCamera>(camera_config, it_);
      cameras_.push_back(camera);
      engine_.addCamera(camera->camera().get());
    }
  }
  else
  {
    mc_rtc::log::warning("No 'Cameras' section defined in Engine3d, only the main camera will be streamed");
  }

  // Every camera frame must exist on the controlled robot
  for(const auto & cam : cameras_)
  {
    if(!gc_->robot().hasFrame(cam->getFrame()))
    {
      mc_rtc::log::error_and_throw("Camera '{}' is attached to unknown robot frame '{}'", cam->name(), cam->getFrame());
    }
  }

  engine3DConfig("render_rate", render_rate_);

  if(engine3DConfig.has("mesh_model"))
  {
    model_path_ = static_cast<std::string>(engine3DConfig("mesh_model"));
  }
  else
  {
    mc_rtc::log::warning("Mesh model has not been defined in the configuration file");
  }

  mc_rtc::log::info("mc_engine3d initialized with {} camera(s)", cameras_.size());
}

void EngineInterface::renderLoop()
{
  // Engine3D (DIRECT) creates its GL context on and renders from the thread that
  // owns it; run() moved the engine and the cameras here before starting.
  engine_.initialize();

  if(!model_path_.empty())
  {
    mc_rtc::log::info("Loading model : {}", model_path_);
    engine_.openModel(QString::fromStdString(model_path_));
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
      engine_.takePicture();

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

void EngineInterface::run()
{
  // Engine3D::DIRECT only dispatches takePicture() on the thread that owns the
  // engine: hand the engine and every camera over to the render thread so the
  // control step below never blocks on a render.
  engine_.moveToThread(&render_thread_);
  for(auto & cam : cameras_)
  {
    cam->camera()->moveToThread(&render_thread_);
  }

  render_running_ = true;
  render_thread_.start();

  const auto dt =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(gc_->timestep()));

  size_t steps = 0;
  auto last_report = std::chrono::steady_clock::now();

  while(gc_->running && rclcpp::ok())
  {
    const auto tic = std::chrono::steady_clock::now();

    if(!gc_->run())
    {
      break;
    }
    ++steps;

    // Publish the latest camera poses to the render thread.
    {
      std::lock_guard<std::mutex> lock(poses_mutex_);
      poses_.poses.resize(cameras_.size());
      for(size_t i = 0; i < cameras_.size(); ++i)
      {
        poses_.poses[i] = gc_->robot().frame(cameras_[i]->getFrame()).position();
      }
      poses_.stamp = node_->now();
      poses_.valid = true;
    }

    if(rclcpp::ok())
    {
      rclcpp::spin_some(node_);
    }

    const auto now = std::chrono::steady_clock::now();
    if(now - last_report >= std::chrono::seconds(5))
    {
      const double elapsed = std::chrono::duration<double>(now - last_report).count();
      mc_rtc::log::info("mc_engine3d control loop at {:.1f} Hz (target {:.1f} Hz)", steps / elapsed,
                        1.0 / gc_->timestep());
      steps = 0;
      last_report = now;
    }

    std::this_thread::sleep_until(tic + dt);
  }

  gc_->running = false;
  render_running_ = false;
  render_thread_.wait();
}
