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
  /**
   * @brief Wrap a camera owned by the engine (typically the main camera)
   *
   * The camera is configured in place and its lifetime stays with the engine.
   *
   * @param camera Camera instance owned by the Engine3D engine
   * @param config Camera configuration
   * @param it Image transport used to advertise the camera stream
   */
  EngineInterfaceCamera(MIS::Camera * camera,
                        const mc_control::Configuration & config,
                        std::shared_ptr<image_transport::ImageTransport> it);

  /**
   * @brief Create a new (secondary) camera from its configuration
   *
   * Ownership of the underlying camera is transferred to the engine through
   * Engine3D::addCamera, hence the wrapper only keeps a non-owning handle.
   *
   * @param config Camera configuration
   * @param it Image transport used to advertise the camera stream
   */
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
   * @param pose Pose of the camera frame expressed in the world frame
   */
  void transform(const sva::PTransformd & pose);

  /**
   * @brief Capture the last rendered camera frame
   *
   * @return cv::Mat BGR image, empty if no frame has been rendered yet
   */
  cv::Mat capture();

  /**
   * @brief Capture and publish frame on ROS topic
   * ROS topic name = camera's name
   *
   * @param stamp Time of the controller state the frame corresponds to; the
   * image is published with the camera frame as header frame_id
   */
  void publish(const rclcpp::Time & stamp);

  /**
   * @brief Get the name of the camera (also used as ROS topic name)
   *
   * @return std::string
   */
  inline std::string name() const
  {
    return cam_->objectName().toStdString();
  }

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
  /// Advertise the camera stream on the image_transport network
  void advertise(std::shared_ptr<image_transport::ImageTransport> it);

  std::shared_ptr<MIS::Camera> cam_;
  image_transport::Publisher pub_;

  std::string frame_;

  /// When set, every captured frame is also written to <save_path>/<name>.png
  std::string save_path_;
};
} // namespace mc_engine3d
