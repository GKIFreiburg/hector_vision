//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_QRCODE_DETECTION_H
#define HECTOR_QRCODE_DETECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

namespace zbar {
  class ImageScanner;
}

namespace hector_qrcode_detection {

class qrcode_detection_impl {
public:
  qrcode_detection_impl(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~qrcode_detection_impl();

protected:
  bool enable_detection(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool disable_detection(std_srvs::Empty::Request &, std_srvs::Empty::Response &);

  void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);

  /// Detect barcodes and publish percepts if found.
  /// \Return true if at least one barcode was found in image.
  bool detectAnPublish(const cv_bridge::CvImageConstPtr & image, const sensor_msgs::CameraInfoConstPtr& camera_info);

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraSubscriber camera_subscriber_;
  image_transport::CameraPublisher rotated_image_publisher_;
  image_transport::CameraPublisher qrcode_image_publisher_;

  ros::ServiceServer srv_enable_;
  ros::ServiceServer srv_disable_;

  ros::Publisher percept_publisher_;

  zbar::ImageScanner *scanner_;

  bool gui_;

  double rotation_min_;
  double rotation_max_;
  double rotation_step_;
  int rotation_image_size_;
};

} // namespace hector_qrcode_detection

#endif // HECTOR_QRCODE_DETECTION_H
