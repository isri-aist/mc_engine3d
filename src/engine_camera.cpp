#include <mc_engine3d/engine_camera.h>
#include <mc_engine3d/engine_config.h>

using namespace mc_engine3d;

namespace
{
/// Non-owning shared_ptr: every camera handled here lives in the engine's
/// camera list and is destroyed by Engine3D, never by this wrapper.
std::shared_ptr<MIS::Camera> wrap(MIS::Camera * camera)
{
  return std::shared_ptr<MIS::Camera>(camera, [](MIS::Camera *) {});
}
} // namespace

EngineInterfaceCamera::EngineInterfaceCamera(MIS::Camera * camera,
                                             const mc_control::Configuration & config,
                                             std::shared_ptr<image_transport::ImageTransport> it)
: cam_(wrap(camera))
{
  loadConfig(config);
  advertise(it);
}

EngineInterfaceCamera::EngineInterfaceCamera(const mc_control::Configuration & config,
                                             std::shared_ptr<image_transport::ImageTransport> it)
: cam_(wrap(new MIS::Camera()))
{
  loadConfig(config);
  advertise(it);
}

void EngineInterfaceCamera::advertise(std::shared_ptr<image_transport::ImageTransport> it)
{
  pub_ = it->advertise(name(), 1);
}

void EngineInterfaceCamera::loadConfig(const mc_control::Configuration & config)
{
  std::string name = "MainCamera";
  if(config.has("name"))
  {
    name = static_cast<std::string>(config("name"));
  }

  if(config.has("frame"))
  {
    frame_ = static_cast<std::string>(config("frame"));
  }
  else
  {
    mc_rtc::log::error_and_throw("No frame specified for camera : {}", name);
  }

  int width = static_cast<int>(config("size")[0]);
  int height = static_cast<int>(config("size")[1]);

  cam_->setObjectName(name.c_str());
  cam_->setActive(true);
  cam_->setSize(width, height);
  cam_->setProjectionType((Camera::ProjectionType) static_cast<int>(config("projection_type")));
  cam_->setAu(config("intrinsics")[0]); // set fx
  cam_->setAv(config("intrinsics")[1]); // set fy
  cam_->setU0(config("intrinsics")[2]); // set cx
  cam_->setV0(config("intrinsics")[3]); // set cy

  if(config.has("near_plane"))
  {
    cam_->setNearPlane(static_cast<double>(config("near_plane")));
  }
  if(config.has("far_plane"))
  {
    cam_->setFarPlane(static_cast<double>(config("far_plane")));
  }
  if(config.has("samples"))
  {
    cam_->setSamples(static_cast<int>(config("samples")));
  }

  std::array<double, 3> background = {0.5, 0.5, 0.5};
  config("background_color", background);
  cam_->setBackgroundColor(background[0], background[1], background[2]);

  config("save_path", save_path_);
}

void EngineInterfaceCamera::transform(const sva::PTransformd & pose)
{
  auto quat = Eigen::Quaterniond(pose.rotation());
  QQuaternion q(quat.w(), quat.x(), quat.y(), quat.z());
  QVector3D v(pose.translation().x(), pose.translation().y(), pose.translation().z());

  QMatrix3x3 cRw = q.toRotationMatrix();
  QMatrix4x4 cMw(cRw(0, 0), cRw(0, 1), cRw(0, 2), v[0], cRw(1, 0), cRw(1, 1), cRw(1, 2), v[1], cRw(2, 0), cRw(2, 1),
                 cRw(2, 2), v[2], 0, 0, 0, 1);

  cMw = wMw * cMw;

  cam_->setcMw(mat4(cMw(0, 0), cMw(1, 0), cMw(2, 0), cMw(3, 0), cMw(0, 1), cMw(1, 1), cMw(2, 1), cMw(3, 1), cMw(0, 2),
                    cMw(1, 2), cMw(2, 2), cMw(3, 2), cMw(0, 3), cMw(1, 3), cMw(2, 3), cMw(3, 3)));
}

cv::Mat EngineInterfaceCamera::capture()
{
  // getFrame() returns the framebuffer cached during the last render; it does
  // not need a current OpenGL context and is safe for the secondary cameras.
  QImage I = cam_->getFrame();
  if(I.isNull())
  {
    return cv::Mat();
  }

  I = I.convertToFormat(QImage::Format_BGR888);

  if(!save_path_.empty())
  {
    I.save(QString::fromStdString(save_path_ + "/" + name() + ".png"));
  }

  // Wrap the QImage buffer accounting for row padding, then copy so the cv::Mat
  // owns its data once the local QImage goes out of scope.
  return cv::Mat(I.height(), I.width(), CV_8UC3, I.bits(), I.bytesPerLine()).clone();
}

void EngineInterfaceCamera::publish(const rclcpp::Time & stamp)
{
  auto img = capture();
  if(!img.empty())
  {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_;
    auto msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub_.publish(msg);
  }
}
