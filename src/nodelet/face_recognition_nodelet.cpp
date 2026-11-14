/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Yuki Furuta.
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
*   * Neither the name of the Kei Okada nor the names of its
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

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <opencv2/opencv.hpp>
#include <pwd.h>
#include <unistd.h>
#include <vector>

#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/version.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

#include <opencv_apps/nodelet.h>
#include <opencv_apps/msg/face_array_stamped.hpp>
#include <opencv_apps/srv/face_recognition_train.hpp>

namespace enc = sensor_msgs::image_encodings;
namespace fs = std::filesystem;

#if CV_MAJOR_VERSION >= 3
#include <opencv2/face.hpp>
namespace face = cv::face;
#else
namespace face = cv;
#endif

#if CV_MAJOR_VERSION >= 4
#include <opencv2/imgcodecs/legacy/constants_c.h>  // include CV_LOAD_IMAGE_COLOR
#include <opencv2/imgproc/imgproc_c.h>             // include CV_AA
#endif

namespace opencv_apps
{
// utility for resolving path
fs::path user_expanded_path(const fs::path& p)
{
  if (p.empty())
    return p;

  auto it = p.begin();
  // Replace the tilda with the home directory
  std::string user_dir = (*it).string();
  if (user_dir.length() == 0 || user_dir[0] != '~')
    return p;
  fs::path ret;
  char* homedir;
  if (user_dir.length() == 1)
  {
    homedir = getenv("HOME");
    if (homedir == nullptr)
    {
      homedir = getpwuid(getuid())->pw_dir;
    }
  }
  else
  {
    std::string uname = user_dir.substr(1, user_dir.length());
    passwd* pw = getpwnam(uname.c_str());
    if (pw == nullptr)
      return p;
    homedir = pw->pw_dir;
  }

  // Append the rest of path
  ret = fs::path(std::string(homedir));
  ++it;
  for (; it != p.end(); ++it)
  {
    ret /= *it;
  }
  return ret;
}

class LabelMapper
{
  std::map<std::string, int> m_;

public:
  void add(std::vector<std::string>& l)
  {
    int id = 0;
    for (std::map<std::string, int>::const_iterator it = m_.begin(); it != m_.end(); ++it)
    {
      if (id < it->second)
        id = it->second + 1;
    }
    for (const std::string& i : l)
    {
      if (m_.find(i) == m_.end())
      {
        m_[i] = id;
        id++;
      }
    }
  }
  std::vector<int> get(std::vector<std::string>& v)
  {
    std::vector<int> ret(v.size());
    for (size_t i = 0; i < v.size(); ++i)
    {
      ret[i] = m_[v[i]];
    }
    return ret;
  }
  std::string lookup(int id)
  {
    for (std::map<std::string, int>::const_iterator it = m_.begin(); it != m_.end(); ++it)
    {
      if (it->second == id)
        return it->first;
    }
    return "nan";
  }
  const std::map<std::string, int>& getMap() const
  {
    return m_;
  }

  void debugPrint()
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("opencv_apps"), "label mapper: debug print:");
    for (std::map<std::string, int>::const_iterator it = m_.begin(); it != m_.end(); ++it)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("opencv_apps"), "\t" << it->first << ": " << it->second);
    }
    RCLCPP_WARN_STREAM(rclcpp::get_logger("opencv_apps"), "label mapper: debug print end");
  }
};

class Storage
{
  fs::path base_dir_;

public:
  Storage(const fs::path& base_dir)
  {
    base_dir_ = user_expanded_path(base_dir);
    if (!fs::exists(base_dir_))
    {
      init();
    }
    if (!fs::is_directory(base_dir_))
    {
      std::stringstream ss;
      ss << base_dir_ << " is not a directory";
      throw std::runtime_error(ss.str());
    }
  };
  void init()
  {
    if (!fs::create_directories(base_dir_))
    {
      std::stringstream ss;
      ss << "failed to initialize directory: " << base_dir_;
      throw std::runtime_error(ss.str());
    }
  }

  void load(std::vector<cv::Mat>& images, std::vector<std::string>& labels, bool append = true)
  {
    if (!append)
    {
      images.clear();
      labels.clear();
    }
    fs::directory_iterator end;
    for (fs::directory_iterator it(base_dir_); it != end; ++it)
    {
      if (fs::is_directory(*it))
      {
        std::string label = it->path().stem().string();
        for (fs::directory_iterator cit(it->path()); cit != end; ++cit)
        {
          if (fs::is_directory(*cit))
            continue;
          fs::path file_path = cit->path();
          try
          {
            cv::Mat img = cv::imread(file_path.string(), CV_LOAD_IMAGE_COLOR);
            labels.push_back(label);
            images.push_back(img);
          }
          catch (cv::Exception& e)
          {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("opencv_apps"),
                                "Error: load image from " << file_path << ": " << e.what());
          }
        }
      }
    }
  }

  void save(const std::vector<cv::Mat>& images, const std::vector<std::string>& labels)
  {
    if (images.size() != labels.size())
    {
      throw std::length_error("images.size() != labels.size()");
    }
    for (size_t i = 0; i < images.size(); ++i)
    {
      save(images[i], labels[i]);
    }
  }

  void save(const cv::Mat& image, const std::string& label)
  {
    fs::path img_dir = base_dir_ / fs::path(label);
    if (!fs::exists(img_dir) && !fs::create_directories(img_dir))
    {
      std::stringstream ss;
      ss << "failed to initialize directory: " << img_dir;
      throw std::runtime_error(ss.str());
    }
    fs::directory_iterator end;
    int file_count = 0;
    for (fs::directory_iterator it(img_dir); it != end; ++it)
    {
      if (!fs::is_directory(*it))
        file_count++;
    }

    std::stringstream ss;
    // data_dir/person_label/person_label_123456.jpg
    ss << label << "_" << std::setw(6) << std::setfill('0') << file_count << ".jpg";
    fs::path file_path = img_dir / ss.str();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("opencv_apps"), "saving image to :" << file_path);
    try
    {
      cv::imwrite(file_path.string(), image);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("opencv_apps"), "Error: save image to " << file_path << ": " << e.what());
    }
  }
};

// parameters formerly defined in cfg/FaceRecognition.cfg
struct FaceRecognitionConfig
{
  std::string model_method;
  bool use_saved_data;
  bool save_train_data;
  std::string data_dir;
  int face_model_width;
  int face_model_height;
  double face_padding;
  int model_num_components;
  double model_threshold;
  int lbph_radius;
  int lbph_neighbors;
  int lbph_grid_x;
  int lbph_grid_y;
};

class FaceRecognitionNodelet : public opencv_apps::Nodelet
{
  typedef opencv_apps::FaceRecognitionConfig Config;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, opencv_apps::msg::FaceArrayStamped>
      SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, opencv_apps::msg::FaceArrayStamped>
      ApproximateSyncPolicy;

  Config config_ = Config();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
  image_transport::SubscriberFilter img_sub_;
  message_filters::Subscriber<opencv_apps::msg::FaceArrayStamped> face_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  rclcpp::Publisher<opencv_apps::msg::FaceArrayStamped>::SharedPtr face_pub_;
  rclcpp::Publisher<opencv_apps::msg::FaceArrayStamped>::SharedPtr faces_pub_;
  rclcpp::Service<opencv_apps::srv::FaceRecognitionTrain>::SharedPtr train_srv_;

  bool save_train_data_;
  bool use_async_;
  bool use_saved_data_;
  double face_padding_;
  int queue_size_;
  std::string data_dir_;
  std::mutex mutex_;

  std::shared_ptr<LabelMapper> label_mapper_;
  std::shared_ptr<Storage> storage_;
  cv::Size face_model_size_;
  cv::Ptr<face::FaceRecognizer> model_;

  void drawFace(cv::Mat& img, const opencv_apps::msg::Face& face)
  {
    cv::Rect r(int(face.face.x - face.face.width / 2.0 - face.face.width * face_padding_ / 2.0),
               int(face.face.y - face.face.height / 2.0 - face.face.height * face_padding_ / 2.0),
               int(face.face.width + face.face.width * face_padding_),
               int(face.face.height + face.face.height * face_padding_));
    cv::Scalar color(0.0, 0.0, 255.0);
    int boldness = 2;
    cv::rectangle(img, r.tl(), r.br(), color, boldness, CV_AA);

    double font_scale = 1.5;
    int text_height = 20;
    cv::Point text_bl;
    if (r.br().y + text_height > img.rows)
      text_bl = r.tl() + cv::Point(0, -text_height);
    else
      text_bl = r.br() + cv::Point(-r.width, text_height);
    std::stringstream ss;
    ss << face.label << " (" << std::fixed << std::setprecision(2) << face.confidence << ")";
    cv::putText(img, ss.str(), text_bl, cv::FONT_HERSHEY_PLAIN, font_scale, color, boldness, CV_AA);
  }

  void extractImage(const cv::Mat& img, const opencv_apps::msg::Rect& rect, cv::Mat& ret, double padding = 0.0)
  {
    int x = std::max(0, int(rect.x - rect.width / 2.0 - rect.width * padding / 2.0));
    int y = std::max(0, int(rect.y - rect.height / 2.0 - rect.height * padding / 2.0));
    cv::Rect r(x, y, std::min(img.cols - x, int(rect.width + rect.width * padding)),
               std::min(img.rows - y, int(rect.height + rect.height * padding)));
    ret = img(r);
  }

  void extractImage(const cv::Mat& img, const opencv_apps::msg::Face& face, cv::Mat& ret, double padding = 0.0)
  {
    extractImage(img, face.face, ret, padding);
  }

  void retrain()
  {
    RCLCPP_DEBUG(get_logger(), "retrain");
    std::vector<cv::Mat> images;
    std::vector<std::string> labels;
    train(images, labels);
  }

  void train(std::vector<cv::Mat>& images, std::vector<std::string>& labels)
  {
    size_t new_image_size = images.size();

    if (use_saved_data_)
    {
      storage_->load(images, labels);
    }

    if (images.empty())
      return;

    std::vector<cv::Mat> resized_images(images.size());
    for (int i = 0; i < images.size(); ++i)
    {
      cv::resize(images[i], resized_images[i], face_model_size_, 0, 0, cv::INTER_CUBIC);
      cv::cvtColor(resized_images[i], resized_images[i], CV_BGR2GRAY);
    }

    label_mapper_->add(labels);
    std::vector<int> ids = label_mapper_->get(labels);
    RCLCPP_INFO_STREAM(get_logger(), "training with " << images.size() << " images");
    // label_mapper_->debugPrint();
    model_->train(resized_images, ids);

    if (save_train_data_ && new_image_size > 0)
    {
      std::vector<cv::Mat> new_images(images.begin(), images.begin() + new_image_size);
      std::vector<std::string> new_labels(labels.begin(), labels.begin() + new_image_size);
      storage_->save(new_images, new_labels);
    }
  }

  void predict(const cv::Mat& img, int& label, double& confidence)
  {
    cv::Mat resized_img;
    cv::resize(img, resized_img, face_model_size_, 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(resized_img, resized_img, CV_BGR2GRAY);
    model_->predict(resized_img, label, confidence);
  }

  void faceImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                         const opencv_apps::msg::FaceArrayStamped::ConstSharedPtr& faces)
  {
    RCLCPP_DEBUG(get_logger(), "faceImageCallback");
    std::lock_guard<std::mutex> lock(mutex_);

    // check if the face data is being trained
    if (label_mapper_->getMap().empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Face data is not trained. Please train first.");
      return;
    }

    // check if need to draw and publish debug image
    bool publish_debug_image = debug_img_pub_->get_subscription_count() > 0;

    try
    {
      cv::Mat cv_img = cv_bridge::toCvShare(image, enc::BGR8)->image;
      opencv_apps::msg::FaceArrayStamped ret_msg = *faces;
      for (size_t i = 0; i < faces->faces.size(); ++i)
      {
        cv::Mat face_img, resized_image;
        extractImage(cv_img, faces->faces[i], face_img, face_padding_);

        int label_id = -1;
        double confidence = 0.0;
        predict(face_img, label_id, confidence);
        if (label_id == -1)
          continue;
        ret_msg.faces[i].label = label_mapper_->lookup(label_id);
        ret_msg.faces[i].confidence = confidence;

        // draw debug image
        if (publish_debug_image)
          drawFace(cv_img, ret_msg.faces[i]);
      }
      face_pub_->publish(ret_msg);
      faces_pub_->publish(ret_msg);
      if (publish_debug_image)
      {
        std::unique_ptr<sensor_msgs::msg::Image> debug_img(new sensor_msgs::msg::Image());
        cv_bridge::CvImage(image->header, enc::BGR8, cv_img).toImageMsg(*debug_img);
        debug_img_pub_->publish(std::move(debug_img));
        RCLCPP_DEBUG(get_logger(), "Published debug image");
      }
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "error at image processing: " << e.err << " " << e.func << " " << e.file << " " << e.line);
    }
  }

  void trainCallback(const std::shared_ptr<opencv_apps::srv::FaceRecognitionTrain::Request> req,
                     std::shared_ptr<opencv_apps::srv::FaceRecognitionTrain::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      std::vector<cv::Mat> images(req->images.size());
      bool use_roi = !req->rects.empty();

      if (use_roi && req->images.size() != req->rects.size())
      {
        throw std::length_error("req.images.size() != req.rects.size()");
      }

      for (size_t i = 0; i < req->images.size(); ++i)
      {
        sensor_msgs::msg::Image img = req->images[i];
        images[i] = cv_bridge::toCvCopy(img, enc::BGR8)->image;
        if (use_roi)
        {
          cv::Mat face_img;
          extractImage(images[i], req->rects[i], face_img);
          images[i] = face_img;
        }
      }
      std::vector<std::string> labels(req->labels.begin(), req->labels.end());
      train(images, labels);
      res->ok = true;
    }
    catch (cv::Exception& e)
    {
      std::stringstream ss;
      ss << "error at training: " << e.err << " " << e.func << " " << e.file << " " << e.line;
      res->ok = false;
      res->error = ss.str();
    }
  }

  void configCallback(Config& config)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    bool need_recreate_model = false;
    bool need_retrain = false;

    use_saved_data_ = config.use_saved_data;
    save_train_data_ = config.save_train_data;
    face_padding_ = config.face_padding;

    if (face_model_size_.width != config.face_model_width)
    {
      face_model_size_.width = config.face_model_width;
      need_retrain = true;
    }
    if (face_model_size_.height != config.face_model_height)
    {
      face_model_size_.height = config.face_model_height;
      need_retrain = true;
    }

    if (data_dir_ != config.data_dir)
    {
      data_dir_ = config.data_dir;
      need_retrain = true;
      label_mapper_.reset(new LabelMapper());
      storage_.reset(new Storage(fs::path(data_dir_)));
    }

    if (config_.model_method != config.model_method)
    {
      need_recreate_model = true;
    }

    if (config_.model_num_components != config.model_num_components)
    {
      need_recreate_model = true;
    }

    if (config.model_method == "LBPH" &&
        (config_.lbph_radius != config.lbph_radius || config_.lbph_neighbors != config.lbph_neighbors ||
         config_.lbph_grid_x != config.lbph_grid_x || config_.lbph_grid_y != config.lbph_grid_y))
    {
      need_recreate_model = true;
    }

    if (need_recreate_model)
    {
      try
      {
        if (config.model_method == "eigen")
        {
// https://docs.opencv.org/3.3.1/da/d60/tutorial_face_main.html
#if (CV_MAJOR_VERSION >= 4) || (CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 3)
          model_ = face::EigenFaceRecognizer::create(config.model_num_components, config.model_threshold);
#else
          model_ = face::createEigenFaceRecognizer(config.model_num_components, config.model_threshold);
#endif
        }
        else if (config.model_method == "fisher")
        {
#if (CV_MAJOR_VERSION >= 4) || (CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 3)
          model_ = face::FisherFaceRecognizer::create(config.model_num_components, config.model_threshold);
#else
          model_ = face::createFisherFaceRecognizer(config.model_num_components, config.model_threshold);
#endif
        }
        else if (config.model_method == "LBPH")
        {
#if (CV_MAJOR_VERSION >= 4) || (CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 3)
          model_ = face::LBPHFaceRecognizer::create(config.lbph_radius, config.lbph_neighbors, config.lbph_grid_x,
                                                    config.lbph_grid_y);
#else
          model_ = face::createLBPHFaceRecognizer(config.lbph_radius, config.lbph_neighbors, config.lbph_grid_x,
                                                  config.lbph_grid_y);
#endif
        }
        need_retrain = true;
      }
      catch (cv::Exception& e)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Error: create face recognizer: " << e.what());
      }
    }

    if (need_retrain)
    {
      try
      {
        retrain();
      }
      catch (cv::Exception& e)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Error: train: " << e.what());
      }
    }

    if (config_.model_threshold != config.model_threshold)
    {
      try
      {
#if CV_MAJOR_VERSION >= 3
        if (face::BasicFaceRecognizer* model = dynamic_cast<face::BasicFaceRecognizer*>(model_.get()))
        {
          model->setThreshold(config.model_threshold);
        }
        else if (face::LBPHFaceRecognizer* model = dynamic_cast<face::LBPHFaceRecognizer*>(model_.get()))
        {
          model->setThreshold(config.model_threshold);
        }
#else
        model_->set("threshold", config.model_threshold);
#endif
      }
      catch (cv::Exception& e)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Error: set threshold: " << e.what());
      }
    }
    config_ = config;
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    Config config = config_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "model_method")
        config.model_method = parameter.as_string();
      else if (parameter.get_name() == "use_saved_data")
        config.use_saved_data = parameter.as_bool();
      else if (parameter.get_name() == "save_train_data")
        config.save_train_data = parameter.as_bool();
      else if (parameter.get_name() == "data_dir")
        config.data_dir = parameter.as_string();
      else if (parameter.get_name() == "face_model_width")
        config.face_model_width = parameter.as_int();
      else if (parameter.get_name() == "face_model_height")
        config.face_model_height = parameter.as_int();
      else if (parameter.get_name() == "face_padding")
        config.face_padding = parameter.as_double();
      else if (parameter.get_name() == "model_num_components")
        config.model_num_components = parameter.as_int();
      else if (parameter.get_name() == "model_threshold")
        config.model_threshold = parameter.as_double();
      else if (parameter.get_name() == "lbph_radius")
        config.lbph_radius = parameter.as_int();
      else if (parameter.get_name() == "lbph_neighbors")
        config.lbph_neighbors = parameter.as_int();
      else if (parameter.get_name() == "lbph_grid_x")
        config.lbph_grid_x = parameter.as_int();
      else if (parameter.get_name() == "lbph_grid_y")
        config.lbph_grid_y = parameter.as_int();
    }
    configCallback(config);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    // image_transport takes rclcpp::QoS since lyrical, message_filters since kilted
#if __has_include(<image_transport/node_interfaces.hpp>)
    img_sub_.subscribe(*this, "image", "raw", subscriberQos(1));
#else
    img_sub_.subscribe(this, "image", "raw", subscriberQos(1).get_rmw_qos_profile());
#endif
#if RCLCPP_VERSION_GTE(29, 0, 0)
    face_sub_.subscribe(this, "faces", subscriberQos(1));
#else
    face_sub_.subscribe(this, "faces", subscriberQos(1).get_rmw_qos_profile());
#endif
    if (use_async_)
    {
      async_ =
          std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(ApproximateSyncPolicy(queue_size_));
      async_->connectInput(img_sub_, face_sub_);
      async_->registerCallback(
          std::bind(&FaceRecognitionNodelet::faceImageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(queue_size_));
      sync_->connectInput(img_sub_, face_sub_);
      sync_->registerCallback(
          std::bind(&FaceRecognitionNodelet::faceImageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    img_sub_.unsubscribe();
    face_sub_.unsubscribe();
  }

public:
  ~FaceRecognitionNodelet() override
  {
    sync_.reset();
    async_.reset();
  }

  explicit FaceRecognitionNodelet(const rclcpp::NodeOptions& options) : Nodelet("face_recognition", options)
  {
    // variables
    face_model_size_ = cv::Size(190, 90);

    // parameters (formerly dynamic reconfigure)
    Config config;
    config.model_method = declare_parameter(
        "model_method", std::string("eigen"),
        paramDescriptor("Method to recognize faces, \"eigen\": eigen, \"fisher\": fisher, \"LBPH\": LBPH"));
    config.use_saved_data = declare_parameter("use_saved_data", true, paramDescriptor("Use saved data"));
    config.save_train_data = declare_parameter("save_train_data", true, paramDescriptor("Save train data"));
    config.data_dir = declare_parameter("data_dir", std::string("~/.ros/opencv_apps/face_data"),
                                        paramDescriptor("Save directory for train data"));
    config.face_model_width =
        declare_parameter("face_model_width", 190, paramDescriptor("Width of training face image", 30, 500));
    config.face_model_height =
        declare_parameter("face_model_height", 90, paramDescriptor("Height of training face image", 30, 500));
    config.face_padding =
        declare_parameter("face_padding", 0.1, paramDescriptor("Padding ratio of each face", 0.0, 2.0));
    config.model_num_components = declare_parameter(
        "model_num_components", 0, paramDescriptor("Number of components for face recognizer model", 0, 100));
    config.model_threshold = declare_parameter("model_threshold", 8000.0,
                                               paramDescriptor("Threshold for face recognizer model", 0.0, 10000.0));
    config.lbph_radius =
        declare_parameter("lbph_radius", 1, paramDescriptor("Radius parameter used only for LBPH model", 1, 10));
    config.lbph_neighbors =
        declare_parameter("lbph_neighbors", 8, paramDescriptor("Neighbors parameter used only for LBPH model", 1, 30));
    config.lbph_grid_x =
        declare_parameter("lbph_grid_x", 8, paramDescriptor("grid_x parameter used only for LBPH model", 1, 30));
    config.lbph_grid_y =
        declare_parameter("lbph_grid_y", 8, paramDescriptor("grid_y parameter used only for LBPH model", 1, 30));
    configCallback(config);
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&FaceRecognitionNodelet::parametersCallback, this, std::placeholders::_1));

    // parameters
    use_async_ = declare_parameter("approximate_sync", false);
    queue_size_ = declare_parameter("queue_size", 100);

    // advertise
    debug_img_pub_ = advertise<sensor_msgs::msg::Image>("~/debug_image", 1);
    // ~/faces is the same as ~/output, with the name of the topic of the other face nodes
    face_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/output", 1);
    faces_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/faces", 1);
    train_srv_ = create_service<opencv_apps::srv::FaceRecognitionTrain>(
        "~/train",
        std::bind(&FaceRecognitionNodelet::trainCallback, this, std::placeholders::_1, std::placeholders::_2));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::FaceRecognitionNodelet)
