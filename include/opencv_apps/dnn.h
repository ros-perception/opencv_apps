// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Kei Okada.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef OPENCV_APPS_DNN_H_
#define OPENCV_APPS_DNN_H_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace opencv_apps
{
/** @brief
 * Path of a model file. When model is a URL, it is downloaded to $ROS_HOME/opencv_apps/models/
 * (~/.ros/opencv_apps/models/) at the first time, and the downloaded file is used after that.
 *
 * @param model path or URL (http:// or https://) of the model file.
 * @param logger logger to print the download progress.
 * @return path of the model file, or empty when the download failed.
 */
std::string modelFile(const std::string& model, const rclcpp::Logger& logger);

/** @brief
 * cv::dnn backend and target for the name.
 *
 * @param name "auto", "cpu", "opencl", "opencl_fp16", "cuda" or "cuda_fp16". "auto" uses CUDA when OpenCV is built
 * with CUDA and a CUDA device is found, and the CPU otherwise.
 * @param backend cv::dnn::Backend for the name.
 * @param target cv::dnn::Target for the name.
 * @param logger logger to print the selected backend.
 * @return false when the name is unknown, or the backend is not available.
 */
bool dnnBackend(const std::string& name, int& backend, int& target, const rclcpp::Logger& logger);

/** @brief
 * Description of the backend parameter, for paramDescriptor().
 */
extern const char* const DNN_BACKEND_DESCRIPTION;

/** @brief
 * URL of the YuNet face detection model for cv::FaceDetectorYN of this OpenCV version.
 */
extern const char* const YUNET_MODEL_URL;

/** @brief
 * URL of the SFace face recognition model for cv::FaceRecognizerSF.
 */
extern const char* const SFACE_MODEL_URL;
}  // namespace opencv_apps

#endif  // OPENCV_APPS_DNN_H_
