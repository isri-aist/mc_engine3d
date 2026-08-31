#include <mc_engine3d/engine_interface.h>

using namespace mc_engine3d;

EngineInterface::EngineInterface(int argc, char ** argv) : engine_(Engine3D::DIRECT)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  node_ = rclcpp::Node::make_shared("mc_engine3d");
  it_ = std::make_shared<image_transport::ImageTransport>(node_);
}

EngineInterface::~EngineInterface()
{
  // engine_.destroy();
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

  // Create rendering engine in Direct Mode
  engine_.initialize();

  if(engine3DConfig.has("mesh_model"))
  {
    auto filepath = static_cast<std::string>(engine3DConfig("mesh_model"));
    mc_rtc::log::info("Loading model : {}", filepath);
    engine_.openModel(filepath.c_str());
  }
  else
  {
    mc_rtc::log::warning("Mesh model has not been defined in the configuration file");
  }

  mc_rtc::log::info("mc_engine3d initialized with {} camera(s)", cameras_.size());
}

void EngineInterface::run()
{
  while(gc_->running)
  {
    if(!gc_->run())
    {
      gc_->running = false;
      break;
    }

    // Move every camera to its current frame pose before rendering so the
    // captured images match the latest controller state.
    for(auto & cam : cameras_)
    {
      cam->transform(gc_->robot().frame(cam->getFrame()).position());
    }

    // Render once: this fills the framebuffer of the main camera and of every
    // secondary camera (see Engine3D::setFrame).
    engine_.takePicture();

    for(auto & cam : cameras_)
    {
      cam->publish();
    }

    rclcpp::spin_some(node_);
  }
}
