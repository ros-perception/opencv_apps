// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

// https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/video/bg_sub.cpp
// https://github.com/opencv/opencv/blob/3.4/samples/cpp/bgfg_segm.cpp
// https://github.com/opencv/opencv_contrib/blob/3.4/modules/bgsegm/samples/bgfg.cpp
/**
 * Background subtraction: MOG2 and KNN of the video module, and MOG, GMG, CNT, GSOC and LSBP of opencv_contrib's
 * bgsegm module (if available, otherwise they fall back to MOG2).
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#ifdef OPENCV_APPS_HAVE_BGSEGM
#include <opencv2/bgsegm.hpp>
#endif

#include "opencv_apps/nodelet.h"

// CNT is in bgsegm since OpenCV 3.3 (not in melodic's 3.2), GSOC and LSBP since 3.4 (not in kinetic's 3.3)
#if defined(OPENCV_APPS_HAVE_BGSEGM) && (CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 3))
#define OPENCV_APPS_HAVE_BGSEGM_CNT
#endif
#if defined(OPENCV_APPS_HAVE_BGSEGM) && (CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 4))
#define OPENCV_APPS_HAVE_BGSEGM_GSOC_LSBP
#endif

namespace opencv_apps
{
class BackgroundSubtractionNodelet : public opencv_apps::Nodelet
{
  enum Method
  {
    MOG2 = 0,
    KNN = 1,
    MOG = 2,
    GMG = 3,
    CNT = 4,
    GSOC = 5,
    LSBP = 6
  };

  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Publisher bg_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  std::mutex mutex_;
  bool use_camera_info_;
  int method_;
  double learning_rate_;
  // MOG2
  int mog2_history_;
  double mog2_var_threshold_;
  bool mog2_detect_shadows_;
  int mog2_nmixtures_;
  // KNN
  int knn_history_;
  double knn_dist2_threshold_;
  bool knn_detect_shadows_;
  // MOG
  int mog_history_;
  int mog_nmixtures_;
  double mog_background_ratio_;
  double mog_noise_sigma_;
  // GMG
  int gmg_initialization_frames_;
  double gmg_decision_threshold_;
  int gmg_max_features_;
  int gmg_quantization_levels_;
  double gmg_background_prior_;
  int gmg_smoothing_radius_;
  // CNT
  int cnt_min_pixel_stability_;
  int cnt_max_pixel_stability_;
  bool cnt_use_history_;
  // GSOC
  int gsoc_n_samples_;
  double gsoc_replace_rate_;
  double gsoc_propagation_rate_;
  int gsoc_hits_threshold_;
  // LSBP
  int lsbp_n_samples_;
  int lsbp_radius_;
  int lsbp_threshold_;
  int lsbp_min_count_;

  // the model and its method (MOG2 if the requested one is not available)
  cv::Ptr<cv::BackgroundSubtractor> model_;
  int model_method_;
  // size of the images of the model, GSOC and LSBP require the same size
  cv::Size model_size_;
  // create a new model (and lose its history) only for the method and the parameters only set at the creation,
  // otherwise update the parameters of the current model
  bool need_create_;
  bool need_update_;

  static const char* methodName(int method)
  {
    static const char* names[] = { "MOG2", "KNN", "MOG", "GMG", "CNT", "GSOC", "LSBP" };
    return (0 <= method && method <= LSBP) ? names[method] : "unknown";
  }

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &BackgroundSubtractionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &BackgroundSubtractionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  // set the value of a parameter of the method, and request to create or update the model if it is the current one
  template <class T, class V>
  void setParameter(T& variable, V value, int method, bool creation_only)
  {
    if (variable == static_cast<T>(value))
      return;
    variable = static_cast<T>(value);
    if (method != model_method_)
      return;
    if (creation_only)
      need_create_ = true;
    else
      need_update_ = true;
  }

  // the combinations of the values that OpenCV rejects, which would fail on every frame
  static std::string invalidParameters(int cnt_min_pixel_stability, int cnt_max_pixel_stability,
                                       int gmg_smoothing_radius)
  {
    if (cnt_min_pixel_stability >= cnt_max_pixel_stability)
      return "cnt_min_pixel_stability must be smaller than cnt_max_pixel_stability";
    if (gmg_smoothing_radius > 0 && gmg_smoothing_radius % 2 == 0)
      return "gmg_smoothing_radius must be 0 or odd, as it is the kernel size of the median filter";
    return std::string();
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    // check the combination of the new values before setting any of them
    int cnt_min_pixel_stability = cnt_min_pixel_stability_, cnt_max_pixel_stability = cnt_max_pixel_stability_;
    int gmg_smoothing_radius = gmg_smoothing_radius_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "cnt_min_pixel_stability")
        cnt_min_pixel_stability = parameter.as_int();
      else if (parameter.get_name() == "cnt_max_pixel_stability")
        cnt_max_pixel_stability = parameter.as_int();
      else if (parameter.get_name() == "gmg_smoothing_radius")
        gmg_smoothing_radius = parameter.as_int();
    }
    result.reason = invalidParameters(cnt_min_pixel_stability, cnt_max_pixel_stability, gmg_smoothing_radius);
    if (!result.reason.empty())
    {
      result.successful = false;
      return result;
    }
    for (const rclcpp::Parameter& parameter : parameters)
    {
      const std::string& name = parameter.get_name();
      if (name == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (name == "method")
      {
        if (method_ != parameter.as_int())
        {
          method_ = parameter.as_int();
          need_create_ = true;
        }
      }
      else if (name == "learning_rate")
        learning_rate_ = parameter.as_double();
      // MOG2
      else if (name == "mog2_history")
        setParameter(mog2_history_, parameter.as_int(), MOG2, false);
      else if (name == "mog2_var_threshold")
        setParameter(mog2_var_threshold_, parameter.as_double(), MOG2, false);
      else if (name == "mog2_detect_shadows")
        setParameter(mog2_detect_shadows_, parameter.as_bool(), MOG2, false);
      else if (name == "mog2_nmixtures")  // the model is allocated for the number of mixtures
        setParameter(mog2_nmixtures_, parameter.as_int(), MOG2, true);
      // KNN
      else if (name == "knn_history")
        setParameter(knn_history_, parameter.as_int(), KNN, false);
      else if (name == "knn_dist2_threshold")
        setParameter(knn_dist2_threshold_, parameter.as_double(), KNN, false);
      else if (name == "knn_detect_shadows")
        setParameter(knn_detect_shadows_, parameter.as_bool(), KNN, false);
      // MOG
      else if (name == "mog_history")
        setParameter(mog_history_, parameter.as_int(), MOG, false);
      else if (name == "mog_nmixtures")  // the model is allocated for the number of mixtures
        setParameter(mog_nmixtures_, parameter.as_int(), MOG, true);
      else if (name == "mog_background_ratio")
        setParameter(mog_background_ratio_, parameter.as_double(), MOG, false);
      else if (name == "mog_noise_sigma")
        setParameter(mog_noise_sigma_, parameter.as_double(), MOG, false);
      // GMG
      else if (name == "gmg_initialization_frames")
        setParameter(gmg_initialization_frames_, parameter.as_int(), GMG, false);
      else if (name == "gmg_decision_threshold")
        setParameter(gmg_decision_threshold_, parameter.as_double(), GMG, false);
      else if (name == "gmg_max_features")  // the histograms are allocated for the number of features
        setParameter(gmg_max_features_, parameter.as_int(), GMG, true);
      else if (name == "gmg_quantization_levels")  // the features in the histograms depend on the levels
        setParameter(gmg_quantization_levels_, parameter.as_int(), GMG, true);
      else if (name == "gmg_background_prior")
        setParameter(gmg_background_prior_, parameter.as_double(), GMG, false);
      else if (name == "gmg_smoothing_radius")
        setParameter(gmg_smoothing_radius_, parameter.as_int(), GMG, false);
      // CNT
      // the stability counters of the pixels are kept by the setters, which are incompatible with a lower minimum or
      // without the history, so the model is recreated
      else if (name == "cnt_min_pixel_stability")
        setParameter(cnt_min_pixel_stability_, parameter.as_int(), CNT, true);
      else if (name == "cnt_max_pixel_stability")
        setParameter(cnt_max_pixel_stability_, parameter.as_int(), CNT, false);
      else if (name == "cnt_use_history")
        setParameter(cnt_use_history_, parameter.as_bool(), CNT, true);
      // GSOC and LSBP do not have setters
      else if (name == "gsoc_n_samples")
        setParameter(gsoc_n_samples_, parameter.as_int(), GSOC, true);
      else if (name == "gsoc_replace_rate")
        setParameter(gsoc_replace_rate_, parameter.as_double(), GSOC, true);
      else if (name == "gsoc_propagation_rate")
        setParameter(gsoc_propagation_rate_, parameter.as_double(), GSOC, true);
      else if (name == "gsoc_hits_threshold")
        setParameter(gsoc_hits_threshold_, parameter.as_int(), GSOC, true);
      else if (name == "lsbp_n_samples")
        setParameter(lsbp_n_samples_, parameter.as_int(), LSBP, true);
      else if (name == "lsbp_radius")
        setParameter(lsbp_radius_, parameter.as_int(), LSBP, true);
      else if (name == "lsbp_threshold")
        setParameter(lsbp_threshold_, parameter.as_int(), LSBP, true);
      else if (name == "lsbp_min_count")
        setParameter(lsbp_min_count_, parameter.as_int(), LSBP, true);
    }
    result.successful = true;
    return result;
  }

  // create a new model of method_ with all the parameters (setNMixtures() and setMaxFeatures() are safe only before
  // the first apply(), which allocates the model)
  void createModel()
  {
    model_method_ = method_;
    switch (method_)
    {
#ifdef OPENCV_APPS_HAVE_BGSEGM
      case MOG:
        model_ = cv::bgsegm::createBackgroundSubtractorMOG(mog_history_, mog_nmixtures_, mog_background_ratio_,
                                                           mog_noise_sigma_);
        break;
      case GMG:
      {
        cv::Ptr<cv::bgsegm::BackgroundSubtractorGMG> gmg =
            cv::bgsegm::createBackgroundSubtractorGMG(gmg_initialization_frames_, gmg_decision_threshold_);
        gmg->setMaxFeatures(gmg_max_features_);
        gmg->setQuantizationLevels(gmg_quantization_levels_);
        model_ = gmg;
        break;
      }
#endif
#ifdef OPENCV_APPS_HAVE_BGSEGM_CNT
      case CNT:
        model_ = cv::bgsegm::createBackgroundSubtractorCNT(cnt_min_pixel_stability_, cnt_use_history_,
                                                           cnt_max_pixel_stability_);
        break;
#endif
#ifdef OPENCV_APPS_HAVE_BGSEGM_GSOC_LSBP
      case GSOC:
        model_ = cv::bgsegm::createBackgroundSubtractorGSOC(cv::bgsegm::LSBP_CAMERA_MOTION_COMPENSATION_NONE,
                                                            gsoc_n_samples_, gsoc_replace_rate_, gsoc_propagation_rate_,
                                                            gsoc_hits_threshold_);
        break;
      case LSBP:
        // Tlower, Tupper, Tinc, Tdec, Rscale, Rincdec, noiseRemovalThresholdFacBG/FG are the defaults
        model_ =
            cv::bgsegm::createBackgroundSubtractorLSBP(cv::bgsegm::LSBP_CAMERA_MOTION_COMPENSATION_NONE,
                                                       lsbp_n_samples_, lsbp_radius_, 2.0f, 32.0f, 1.0f, 0.05f, 10.0f,
                                                       0.005f, 0.0004f, 0.0008f, lsbp_threshold_, lsbp_min_count_);
        break;
#endif
      case KNN:
        model_ = cv::createBackgroundSubtractorKNN(knn_history_, knn_dist2_threshold_, knn_detect_shadows_);
        break;
      case MOG2:
        model_ = cv::createBackgroundSubtractorMOG2(mog2_history_, mog2_var_threshold_, mog2_detect_shadows_);
        dynamic_cast<cv::BackgroundSubtractorMOG2*>(model_.get())->setNMixtures(mog2_nmixtures_);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "%s background subtraction is not available (OpenCV %s, the bgsegm module of opencv_contrib is "
                     "required), use MOG2",
                     methodName(method_), CV_VERSION);
        model_method_ = MOG2;
        model_ = cv::createBackgroundSubtractorMOG2(mog2_history_, mog2_var_threshold_, mog2_detect_shadows_);
        dynamic_cast<cv::BackgroundSubtractorMOG2*>(model_.get())->setNMixtures(mog2_nmixtures_);
        break;
    }
    // the parameters that are not arguments of the create functions
    updateModel();
    RCLCPP_INFO(get_logger(), "Create %s background subtractor", methodName(model_method_));
  }

  // update the parameters of the current model, without losing its history
  void updateModel()
  {
    switch (model_method_)
    {
      case MOG2:
      {
        cv::BackgroundSubtractorMOG2* mog2 = dynamic_cast<cv::BackgroundSubtractorMOG2*>(model_.get());
        mog2->setHistory(mog2_history_);
        mog2->setVarThreshold(mog2_var_threshold_);
        mog2->setDetectShadows(mog2_detect_shadows_);
        break;
      }
      case KNN:
      {
        cv::BackgroundSubtractorKNN* knn = dynamic_cast<cv::BackgroundSubtractorKNN*>(model_.get());
        knn->setHistory(knn_history_);
        knn->setDist2Threshold(knn_dist2_threshold_);
        knn->setDetectShadows(knn_detect_shadows_);
        break;
      }
#ifdef OPENCV_APPS_HAVE_BGSEGM
      case MOG:
      {
        cv::bgsegm::BackgroundSubtractorMOG* mog = dynamic_cast<cv::bgsegm::BackgroundSubtractorMOG*>(model_.get());
        mog->setHistory(mog_history_);
        mog->setBackgroundRatio(mog_background_ratio_);
        // 0 is 15 (the default) in createBackgroundSubtractorMOG(), but setNoiseSigma() takes it as it is
        mog->setNoiseSigma(mog_noise_sigma_ > 0 ? mog_noise_sigma_ : 15.0);
        break;
      }
      case GMG:
      {
        cv::bgsegm::BackgroundSubtractorGMG* gmg = dynamic_cast<cv::bgsegm::BackgroundSubtractorGMG*>(model_.get());
        gmg->setNumFrames(gmg_initialization_frames_);
        gmg->setDecisionThreshold(gmg_decision_threshold_);
        gmg->setBackgroundPrior(gmg_background_prior_);
        gmg->setSmoothingRadius(gmg_smoothing_radius_);
        break;
      }
#endif
#ifdef OPENCV_APPS_HAVE_BGSEGM_CNT
      case CNT:
      {
        cv::bgsegm::BackgroundSubtractorCNT* cnt = dynamic_cast<cv::bgsegm::BackgroundSubtractorCNT*>(model_.get());
        // OpenCV requires min < max at each setter, so raise the maximum first when the minimum reaches it
        if (cnt_min_pixel_stability_ >= cnt->getMaxPixelStability())
        {
          cnt->setMaxPixelStability(cnt_max_pixel_stability_);
          cnt->setMinPixelStability(cnt_min_pixel_stability_);
        }
        else
        {
          cnt->setMinPixelStability(cnt_min_pixel_stability_);
          cnt->setMaxPixelStability(cnt_max_pixel_stability_);
        }
        cnt->setUseHistory(cnt_use_history_);
        break;
      }
#endif
      default:
        break;
    }
  }

  // MOG and GMG do not implement getBackgroundImage()
  bool hasBackgroundImage() const
  {
    return model_method_ != MOG && model_method_ != GMG;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      // e.g. image_publisher publishes an empty image when it rewinds the video, which would reinitialize (MOG2,
      // KNN, ...) or break (GSOC, LSBP) the model of the image size
      if (frame.empty())
      {
        RCLCPP_DEBUG(get_logger(), "Skip an empty image");
        return;
      }
      cv::Mat fg_mask, background;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // a new model for the new image size, as GSOC and LSBP require the size of the model
        if (need_create_ || !model_ || frame.size() != model_size_)
        {
          createModel();
          model_size_ = frame.size();
        }
        else if (need_update_)
          updateModel();
        need_create_ = need_update_ = false;

        // 255: foreground, 127: shadow (MOG2 and KNN with detect_shadows), 0: background
        // GMG takes only -1 as the automatic learning rate, which keeps the last explicit rate, so its default
        // (0.025) is given to resume it
        double learning_rate = learning_rate_ >= 0 ? learning_rate_ : (model_method_ == GMG ? 0.025 : -1.0);
        model_->apply(frame, fg_mask, learning_rate);
        if (bg_pub_.getNumSubscribers() > 0 && hasBackgroundImage())
          model_->getBackgroundImage(background);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, fg_mask);
        int c = cv::waitKey(1);
      }

      img_pub_.publish(cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, fg_mask).toImageMsg());
      if (!background.empty())
      {
        // CNT has a gray background image
        bg_pub_.publish(cv_bridge::CvImage(image_msg->header,
                                           background.channels() == 1 ? sensor_msgs::image_encodings::MONO8 :
                                                                        sensor_msgs::image_encodings::BGR8,
                                           background)
                            .toImageMsg());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
  }

public:
  explicit BackgroundSubtractionNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("background_subtraction", options), model_method_(-1), need_create_(true), need_update_(false)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    window_name_ = "BackgroundSubtraction Demo";

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    method_ = declare_parameter(
        "method", 0,
        paramDescriptor("Background subtraction method, 0: MOG2, 1: KNN, 2: MOG, 3: GMG, 4: CNT, 5: GSOC, 6: LSBP "
                        "(MOG, GMG, CNT, GSOC and LSBP need the bgsegm module of opencv_contrib, otherwise MOG2 is "
                        "used)",
                        0, 6));
    learning_rate_ = declare_parameter(
        "learning_rate", -1.0,
        paramDescriptor("Learning rate of the background model, negative: automatically chosen, 0: not updated, 1: "
                        "reinitialized from the last frame (CNT, GSOC and LSBP may still update the model at 0, and "
                        "GMG, GSOC and LSBP do not reinitialize it at 1)",
                        -1.0, 1.0));
    mog2_history_ = declare_parameter("mog2_history", 500, paramDescriptor("MOG2: Length of the history", 1, 5000));
    mog2_var_threshold_ = declare_parameter(
        "mog2_var_threshold", 16.0,
        paramDescriptor("MOG2: Threshold on the squared Mahalanobis distance between the pixel and the model to decide "
                        "whether a pixel is well described by the background model",
                        1.0, 256.0));
    mog2_detect_shadows_ =
        declare_parameter("mog2_detect_shadows", true,
                          paramDescriptor("MOG2: Detect shadows and mark them (127) in the foreground mask"));
    mog2_nmixtures_ = declare_parameter(
        "mog2_nmixtures", 5,
        paramDescriptor("MOG2: Number of Gaussian components in the background model (recreates the model)", 1, 10));
    knn_history_ = declare_parameter("knn_history", 500, paramDescriptor("KNN: Length of the history", 1, 5000));
    knn_dist2_threshold_ = declare_parameter(
        "knn_dist2_threshold", 400.0,
        paramDescriptor("KNN: Threshold on the squared distance between the pixel and the sample to decide whether a "
                        "pixel is close to that sample",
                        1.0, 5000.0));
    knn_detect_shadows_ = declare_parameter(
        "knn_detect_shadows", true, paramDescriptor("KNN: Detect shadows and mark them (127) in the foreground mask"));
    mog_history_ = declare_parameter("mog_history", 200, paramDescriptor("MOG: Length of the history", 1, 5000));
    mog_nmixtures_ = declare_parameter(
        "mog_nmixtures", 5,
        paramDescriptor("MOG: Number of Gaussian mixtures, at most 8 by OpenCV (recreates the model)", 1, 8));
    mog_background_ratio_ =
        declare_parameter("mog_background_ratio", 0.7, paramDescriptor("MOG: Background ratio", 0.0, 1.0));
    mog_noise_sigma_ =
        declare_parameter("mog_noise_sigma", 0.0, paramDescriptor("MOG: Noise strength, 0: automatic", 0.0, 100.0));
    gmg_initialization_frames_ =
        declare_parameter("gmg_initialization_frames", 120,
                          paramDescriptor("GMG: Number of frames used to initialize the background models", 1, 1000));
    gmg_decision_threshold_ = declare_parameter(
        "gmg_decision_threshold", 0.8,
        paramDescriptor("GMG: Threshold of the probability above which a pixel is foreground", 0.0, 1.0));
    gmg_max_features_ = declare_parameter(
        "gmg_max_features", 64,
        paramDescriptor("GMG: Total number of distinct colors to maintain in histogram (recreates the model)", 1, 512));
    gmg_quantization_levels_ = declare_parameter(
        "gmg_quantization_levels", 16,
        paramDescriptor("GMG: Number of discrete levels in each channel to be used in histograms (recreates the model)",
                        1, 128));
    gmg_background_prior_ = declare_parameter(
        "gmg_background_prior", 0.8,
        paramDescriptor("GMG: Prior probability that any given pixel is a background pixel", 0.0, 1.0));
    gmg_smoothing_radius_ = declare_parameter(
        "gmg_smoothing_radius", 7,
        paramDescriptor("GMG: Kernel size of the median filter to clean up the foreground mask, 0 (no filter) or odd",
                        0, 31));
    cnt_min_pixel_stability_ =
        declare_parameter("cnt_min_pixel_stability", 15,
                          paramDescriptor("CNT: Number of frames with same pixel color to consider stable", 1, 1000));
    cnt_max_pixel_stability_ =
        declare_parameter("cnt_max_pixel_stability", 900,
                          paramDescriptor("CNT: Maximum allowed credit for a pixel in history", 1, 10000));
    cnt_use_history_ = declare_parameter("cnt_use_history", true,
                                         paramDescriptor("CNT: Give a pixel credit for being stable for a long time"));
    gsoc_n_samples_ = declare_parameter(
        "gsoc_n_samples", 20,
        paramDescriptor("GSOC: Number of samples to maintain at each point of the frame (recreates the model)", 2, 100));
    gsoc_replace_rate_ = declare_parameter(
        "gsoc_replace_rate", 0.003,
        paramDescriptor("GSOC: Probability of replacing the old sample (recreates the model)", 0.0, 1.0));
    gsoc_propagation_rate_ = declare_parameter(
        "gsoc_propagation_rate", 0.01,
        paramDescriptor("GSOC: Probability of propagating to neighbors (recreates the model)", 0.0, 1.0));
    gsoc_hits_threshold_ = declare_parameter(
        "gsoc_hits_threshold", 32,
        paramDescriptor("GSOC: How many positives the sample must get before it will be considered as a possible "
                        "replacement (recreates the model)",
                        1, 1000));
    lsbp_n_samples_ = declare_parameter(
        "lsbp_n_samples", 20,
        paramDescriptor("LSBP: Number of samples to maintain at each point of the frame (recreates the model)", 2, 100));
    lsbp_radius_ = declare_parameter(
        "lsbp_radius", 16, paramDescriptor("LSBP: Radius of the LSBP descriptor (recreates the model)", 1, 64));
    lsbp_threshold_ = declare_parameter(
        "lsbp_threshold", 8,
        paramDescriptor("LSBP: Threshold of the LSBP descriptor distance (recreates the model)", 0, 32));
    lsbp_min_count_ = declare_parameter(
        "lsbp_min_count", 2,
        paramDescriptor("LSBP: Minimal number of matches for a pixel to be background (recreates the model)", 1, 100));
    // the defaults for the invalid combinations given at the start
    std::string invalid = invalidParameters(cnt_min_pixel_stability_, cnt_max_pixel_stability_, gmg_smoothing_radius_);
    if (!invalid.empty())
    {
      RCLCPP_ERROR(get_logger(), "%s, use the defaults", invalid.c_str());
      if (cnt_min_pixel_stability_ >= cnt_max_pixel_stability_)
      {
        cnt_min_pixel_stability_ = 15;
        cnt_max_pixel_stability_ = 900;
      }
      if (gmg_smoothing_radius_ > 0 && gmg_smoothing_radius_ % 2 == 0)
        gmg_smoothing_radius_ = 7;
      // report the values in use, before the callback is registered
      set_parameters({ rclcpp::Parameter("cnt_min_pixel_stability", cnt_min_pixel_stability_),
                       rclcpp::Parameter("cnt_max_pixel_stability", cnt_max_pixel_stability_),
                       rclcpp::Parameter("gmg_smoothing_radius", gmg_smoothing_radius_) });
    }
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&BackgroundSubtractionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    bg_pub_ = advertiseImage("~/background", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::BackgroundSubtractionNodelet)
