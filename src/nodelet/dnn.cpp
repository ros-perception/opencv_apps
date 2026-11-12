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

#include "opencv_apps/dnn.h"

#ifdef OPENCV_APPS_HAVE_CURL
#include <curl/curl.h>
#endif
#include <unistd.h>

#include <opencv2/core/cuda.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace opencv_apps
{
const char* const DNN_BACKEND_DESCRIPTION =
    "Backend of cv::dnn: auto, cpu, opencl, opencl_fp16, cuda or cuda_fp16. auto uses CUDA when OpenCV is built "
    "with CUDA and a CUDA device is found, and the CPU otherwise.";

// the models of https://github.com/opencv/opencv_zoo, the commits are fixed not to change the files
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
const char* const YUNET_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/f12e12798e8314f7c074a6656816c048dcc95b7a/"
                                    "models/face_detection_yunet/face_detection_yunet_2023mar.onnx";
#else
// the 2023mar model requires OpenCV 4.8 or later
const char* const YUNET_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/c97242ce7f2a554e288b50eabd9f5df957e78801/"
                                    "models/face_detection_yunet/face_detection_yunet_2022mar.onnx";
#endif
const char* const SFACE_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/ba91a3b91d00d76e86540d4013f944bd6b514e39/"
                                    "models/face_recognition_sface/face_recognition_sface_2021dec.onnx";

namespace
{
fs::path rosHome()
{
  const char* ros_home = std::getenv("ROS_HOME");
  if (ros_home != nullptr && *ros_home != '\0')
    return fs::path(ros_home);
  const char* home = std::getenv("HOME");
  return fs::path(home != nullptr ? home : ".") / ".ros";
}

#ifdef OPENCV_APPS_HAVE_CURL
size_t writeToFile(void* ptr, size_t size, size_t nmemb, void* stream)
{
  return fwrite(ptr, size, nmemb, static_cast<FILE*>(stream));
}

bool download(const std::string& url, const fs::path& path, const rclcpp::Logger& logger)
{
  static std::once_flag curl_initialized;
  std::call_once(curl_initialized, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });

  // write to a temporary file and rename it, as another process may download the same file at the same time
  fs::path tmp_path = path;
  tmp_path += "." + std::to_string(getpid()) + ".tmp";
  FILE* fp = fopen(tmp_path.c_str(), "wb");
  if (fp == nullptr)
  {
    RCLCPP_ERROR(logger, "Failed to open %s", tmp_path.c_str());
    return false;
  }
  char error[CURL_ERROR_SIZE] = "";
  CURL* curl = curl_easy_init();
  if (curl == nullptr)
  {
    RCLCPP_ERROR(logger, "Failed to initialize curl to download %s", url.c_str());
    fclose(fp);
    fs::remove(tmp_path);
    return false;
  }
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeToFile);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
  // do not wait forever for a stalled server, as the download blocks the other nodelets
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 30L);
  curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
  curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 60L);
  CURLcode result = curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  // the last buffered write may fail at fclose(), e.g. on a full disk
  bool closed = fclose(fp) == 0;
  if (result != CURLE_OK || !closed)
  {
    RCLCPP_ERROR(logger, "Failed to download %s: %s", url.c_str(),
                 !closed ? "failed to write the file" : (error[0] ? error : curl_easy_strerror(result)));
    fs::remove(tmp_path);
    return false;
  }
  std::error_code ec;
  fs::rename(tmp_path, path, ec);
  if (ec)
  {
    fs::remove(tmp_path, ec);
    // another process may have downloaded it at the same time
    if (fs::exists(path, ec))
      return true;
    RCLCPP_ERROR(logger, "Failed to save %s: %s", path.c_str(), ec.message().c_str());
    return false;
  }
  return true;
}
#else
bool download(const std::string& url, const fs::path& path, const rclcpp::Logger& logger)
{
  RCLCPP_ERROR(logger, "opencv_apps is built without libcurl, download %s to %s, or give the model file", url.c_str(),
               path.c_str());
  return false;
}
#endif
// the file to save the URL (without the scheme) in, not to mix the files of the same name, e.g. of other versions.
// The files are under the directory of the scheme (http or https), not to use a model downloaded by http:// for
// https://, and not to use the files of the older versions, which were saved without the scheme.
// The query is replaced with its hash to keep the extension, which OpenCV uses to find the format of the model.
// Returns false for the URL which would be saved out of the models directory, e.g. with "..".
bool cachePath(const std::string& scheme, const std::string& url, fs::path& path)
{
  std::string location = url.substr(0, url.find_first_of("?#"));
  std::string query = url.substr(location.size());
  fs::path relative(location);
  if (location.empty() || relative.is_absolute() || !relative.has_filename())
    return false;
  for (const fs::path& part : relative)
  {
    if (part == "." || part == "..")
      return false;
  }
  path = rosHome() / "opencv_apps" / "models" / scheme / relative.parent_path();
  if (!query.empty())
  {
    std::stringstream ss;
    ss << std::hex << std::hash<std::string>()(query);
    path /= ss.str();
  }
  path /= relative.filename();
  return true;
}
}  // namespace

std::string modelFile(const std::string& model, const rclcpp::Logger& logger)
{
  const std::vector<std::string> schemes = { "https", "http" };
  for (const std::string& scheme : schemes)
  {
    const std::string prefix = scheme + "://";
    if (model.compare(0, prefix.size(), prefix) != 0)
      continue;
    fs::path path;
    if (!cachePath(scheme, model.substr(prefix.size()), path))
    {
      RCLCPP_ERROR(logger, "Invalid model URL %s", model.c_str());
      return std::string();
    }
    // error_code not to throw, e.g. for a directory which is not searchable
    std::error_code ec;
    if (fs::exists(path, ec))
      return path.string();

    // nodelets in the same process may use the same file
    static std::mutex download_mutex;
    std::lock_guard<std::mutex> lock(download_mutex);
    if (fs::exists(path, ec))
      return path.string();
    fs::create_directories(path.parent_path(), ec);
    RCLCPP_INFO(logger, "Downloading %s to %s", model.c_str(), path.c_str());
#ifdef OPENCV_APPS_HAVE_CURL
    // retry for temporary network errors
    for (int i = 0; i < 3; i++)
    {
      if (download(model, path, logger))
        return path.string();
      std::this_thread::sleep_for(std::chrono::seconds(1 << i));
    }
#else
    download(model, path, logger);
#endif
    return std::string();
  }
  return model;
}

bool dnnBackend(const std::string& name, int& backend, int& target, const rclcpp::Logger& logger)
{
  bool have_cuda =
      cv::cuda::getCudaEnabledDeviceCount() > 0 && !cv::dnn::getAvailableTargets(cv::dnn::DNN_BACKEND_CUDA).empty();
  std::string selected = name;
  if (name == "auto")
    selected = have_cuda ? "cuda" : "cpu";

  if (selected == "cpu")
  {
    backend = cv::dnn::DNN_BACKEND_OPENCV;
    target = cv::dnn::DNN_TARGET_CPU;
  }
  else if (selected == "opencl" || selected == "opencl_fp16")
  {
    if (!cv::ocl::haveOpenCL())
    {
      RCLCPP_ERROR(logger, "OpenCL is not available for backend %s", name.c_str());
      return false;
    }
    backend = cv::dnn::DNN_BACKEND_OPENCV;
    target = (selected == "opencl") ? cv::dnn::DNN_TARGET_OPENCL : cv::dnn::DNN_TARGET_OPENCL_FP16;
  }
  else if (selected == "cuda" || selected == "cuda_fp16")
  {
    if (!have_cuda)
    {
      RCLCPP_ERROR(logger, "OpenCV is not built with CUDA, or no CUDA device is found, for backend %s", name.c_str());
      return false;
    }
    backend = cv::dnn::DNN_BACKEND_CUDA;
    target = (selected == "cuda") ? cv::dnn::DNN_TARGET_CUDA : cv::dnn::DNN_TARGET_CUDA_FP16;
  }
  else
  {
    RCLCPP_ERROR(logger, "Unknown backend %s, use auto, cpu, opencl, opencl_fp16, cuda or cuda_fp16", name.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "cv::dnn runs on %s", selected.c_str());
  return true;
}
}  // namespace opencv_apps
