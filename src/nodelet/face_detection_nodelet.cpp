/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/objectDetection/objectDetection.cpp
/**
 * @file objectDetection.cpp
 * @author A. Huaman ( based in the classic facedetect.cpp in samples/c )
 * @brief A simplified version of facedetect.cpp, show how to load a cascade classifier and how to find objects (Face + eyes) in a video stream
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/FaceDetectionConfig.h"
#include "opencv_apps/Face.h"
#include "opencv_apps/FaceArray.h"
#include "opencv_apps/FaceArrayStamped.h"

namespace face_detection {
class FaceDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef face_detection::FaceDetectionConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  bool debug_view_;
  ros::Time prev_stamp_;

  cv::CascadeClassifier face_cascade_;
  cv::CascadeClassifier eyes_cascade_;

  void reconfigureCallback(Config &new_config, uint32_t level)
  {
    config_ = new_config;
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    do_work(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::FaceArrayStamped faces_msg;
      faces_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat frame_gray;

      if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
      } else {
        frame_gray = frame;
      }
      cv::equalizeHist( frame_gray, frame_gray );
      //-- Detect faces
#ifndef CV_VERSION_EPOCH
      face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 2, 0, cv::Size(30, 30) );
#else
      face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
#endif

      for( size_t i = 0; i < faces.size(); i++ )
      {
        cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        cv::ellipse( frame,  center, cv::Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 2, 8, 0 );
        opencv_apps::Face face_msg;
        face_msg.face.x = faces[i].x;
        face_msg.face.y = faces[i].y;
        face_msg.face.width = faces[i].width;
        face_msg.face.height = faces[i].height;
        face_msg.face_center.x = center.x;
        face_msg.face_center.y = center.y;

        cv::Mat faceROI = frame_gray( faces[i] );
        std::vector<cv::Rect> eyes;

        //-- In each face, detect eyes
#ifndef CV_VERSION_EPOCH
        eyes_cascade_.detectMultiScale( faceROI, eyes, 1.1, 2, 0, cv::Size(30, 30) );
#else
        eyes_cascade_.detectMultiScale( faceROI, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
#endif

        for( size_t j = 0; j < eyes.size(); j++ )
        {
          cv::Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
          int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
          cv::circle( frame, eye_center, radius, cv::Scalar( 255, 0, 0 ), 3, 8, 0 );

          opencv_apps::Rect eye_msg;
          eye_msg.x = faces[i].x + eyes[j].x;
          eye_msg.y = faces[i].y + eyes[j].y;
          eye_msg.width = eyes[j].width;
          eye_msg.height = eyes[j].height;
          face_msg.eyes.push_back(eye_msg);
          opencv_apps::Point2D eye_center_msg;
          eye_center_msg.x = eye_center.x;
          eye_center_msg.y = eye_center.y;
          face_msg.eyes_center.push_back(eye_center_msg);
        }

        faces_msg.faces.push_back(face_msg);
      }
      //-- Show what you got
      if( debug_view_) {
        cv::imshow( "Face detection", frame );
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding,frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(faces_msg);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &FaceDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &FaceDetectionNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  virtual void onInit()
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("debug_view", debug_view_, false);
    if (debug_view_ ) {
      always_subscribe_ = true;
    }
    prev_stamp_ = ros::Time(0, 0);

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&FaceDetectionNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);
    
    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::FaceArrayStamped>(*pnh_, "faces", 1);

    std::string face_cascade_name, eyes_cascade_name;
    pnh_->param("face_cascade_name", face_cascade_name, std::string("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"));
    pnh_->param("eyes_cascade_name", eyes_cascade_name, std::string("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml"));

    if( !face_cascade_.load( face_cascade_name ) ){ NODELET_ERROR("--Error loading %s", face_cascade_name.c_str()); };
    if( !eyes_cascade_.load( eyes_cascade_name ) ){ NODELET_ERROR("--Error loading %s", eyes_cascade_name.c_str()); };

    onInitPostProcess();
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(face_detection::FaceDetectionNodelet, nodelet::Nodelet);
