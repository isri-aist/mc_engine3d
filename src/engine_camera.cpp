#include <mc_engine3d/engine_camera.h>
#include <mc_engine3d/engine_config.h>

using namespace mc_engine3d;

EngineInterfaceCamera::EngineInterfaceCamera(const mc_control::Configuration & config,
                                             std::shared_ptr<image_transport::ImageTransport> it)
{
  cam_ = std::make_shared<Camera>();
  loadConfig(config);

  pub_ = it->advertise(cam_->objectName().toStdString(), 1);
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
  cam_->setSize(width, height);
  cam_->setProjectionType((Camera::ProjectionType) static_cast<int>(config("projection_type")));
  cam_->setAu(config("intrinsics")[0]); // set fx
  cam_->setAv(config("intrinsics")[1]); // set fy
  cam_->setU0(config("intrinsics")[2]); // set cx
  cam_->setV0(config("intrinsics")[3]); // set cy

  double red = 0.50, green = 0.50, blue = 0.50; // TODO make it better
  cam_->setBackgroundColor(red, green, blue);
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
  std::cout << cam_->objectName().toStdString() << std::endl;
  auto I = cam_->getFrame().convertToFormat(QImage::Format_BGR888);
  I.save("capture" + cam_->objectName() + ".png");
  return cv::Mat(I.height(), I.width(), CV_8UC3, I.bits());
}

void EngineInterfaceCamera::publish()
{
  auto img = capture();
  if(!img.empty())
  {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    pub_.publish(msg);
  }
}