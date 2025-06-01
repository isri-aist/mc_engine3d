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
    auto camConfig = engine3DConfig("MainCamera");

    auto main_camera = std::make_shared<EngineInterfaceCamera>(camConfig, it_);
    cameras_.push_back(main_camera);
    engine_.setMainCamera(main_camera->camera().get());
  }
  else
  {
    mc_rtc::log::error("MainCamera has not been defined in the configuration file");
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
    mc_rtc::log::warning("No 'Cameras' section defined in engine3DConfig.");
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
}

void EngineInterface::run()
{

  while(gc_->running)
  {
    engine_.takePicture();

    if(!gc_->run())
    {
      gc_->running = false;
    }

    for(auto & cam : cameras_)
    {
      std::cout << cam->camera()->objectName().toStdString() << std::endl;
      cam->transform(gc_->robot().frame(cam->getFrame()).position());
      cam->publish();
    }
  }
}