#pragma once

#include <Engine3D/Camera.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <image_transport/publisher.hpp>
#include <mc_control/Configuration.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

using namespace MIS;

namespace mc_engine3d
{

class EngineInterfaceCamera
{
public:
  EngineInterfaceCamera(const mc_control::Configuration & config, std::shared_ptr<image_transport::ImageTransport> it);

  /**
   * @brief Load camera config
   *
   * @param config
   */
  void loadConfig(const mc_control::Configuration & config);

  /**
   * @brief Get Engine3d camera
   *
   * @return std::shared_ptr<Camera>
   */
  inline std::shared_ptr<Camera> camera() const
  {
    return cam_;
  }

  /**
   * @brief Apply transform to camera
   *
   * @param pose
   */
  void transform(const sva::PTransformd & pose);

  /**
   * @brief Capture camera frame
   *
   * @return cv::Mat
   */
  cv::Mat capture();

  /**
   * @brief Capture and publish frame on ROS topic
   * ROS topic name = camera's name
   */
  void publish();

  /**
   * @brief Get the Frame on which the camera is attached
   *
   * @return const std::string&
   */
  const inline std::string & getFrame() const
  {
    return frame_;
  }

private:
  std::shared_ptr<MIS::Camera> cam_;
  image_transport::Publisher pub_;

  std::string frame_;
};
} // namespace mc_engine3d