#include "imx_stereo/dual_camera.hpp"
#include <cv_bridge/cv_bridge.h>

StereoSyncNode::StereoSyncNode() : Node("stereo_sync_node") {
    width_       = this->declare_parameter<int>("width", 640);
    height_      = this->declare_parameter<int>("height", 480);
    fps_         = this->declare_parameter<int>("fps", 30);
    tol_ms_      = this->declare_parameter<int>("tolerance_ms", 10);
    sensor_left_ = this->declare_parameter<int>("sensor_id_left", 1);
    sensor_right_= this->declare_parameter<int>("sensor_id_right", 0);

    exposure_min_ = this->declare_parameter<int>("exposure_min", 100000);
    exposure_max_ = this->declare_parameter<int>("exposure_max", 10000000);
    gain_min_     = this->declare_parameter<double>("gain_min", 1.0);
    gain_max_     = this->declare_parameter<double>("gain_max", 16.0);
    wbmode_       = this->declare_parameter<int>("wbmode", 1);

    left_pub_  = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_sync", 10);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_sync", 10);

    // Init GStreamer
    gst_init(nullptr, nullptr);

    // pipeline
    auto left_desc  = make_pipeline_desc(sensor_left_);
    auto right_desc = make_pipeline_desc(sensor_right_);

    GError *err = nullptr;
    left_pipe_  = gst_parse_launch(left_desc.c_str(),  &err);
    if (err) {
        RCLCPP_FATAL(get_logger(), "Left pipeline error: %s", err->message);
        g_error_free(err);
        throw std::runtime_error("left pipeline");
    }
    right_pipe_ = gst_parse_launch(right_desc.c_str(), &err);
    if (err) {
        RCLCPP_FATAL(get_logger(), "Right pipeline error: %s", err->message);
        g_error_free(err);
        throw std::runtime_error("right pipeline");
    }

    left_sink_  = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(left_pipe_),  "appsink_left"));
    right_sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(right_pipe_), "appsink_right"));

    setup_appsink(left_sink_);
    setup_appsink(right_sink_);

    gst_element_set_state(left_pipe_,  GST_STATE_PLAYING);
    gst_element_set_state(right_pipe_, GST_STATE_PLAYING);

    cap_left_th_  = std::thread(&StereoSyncNode::captureLoop, this, left_sink_,  std::ref(buf_left_),  "L");
    cap_right_th_ = std::thread(&StereoSyncNode::captureLoop, this, right_sink_, std::ref(buf_right_), "R");

    sync_th_ = std::thread(&StereoSyncNode::syncLoop, this);
}

StereoSyncNode::~StereoSyncNode() {
    running_ = false;
  
    if (cap_left_th_.joinable())  cap_left_th_.join();
    if (cap_right_th_.joinable()) cap_right_th_.join();
    if (sync_th_.joinable())      sync_th_.join();
  
    if (left_pipe_)  { gst_element_set_state(left_pipe_,  GST_STATE_NULL); gst_object_unref(left_pipe_); }
    if (right_pipe_) { gst_element_set_state(right_pipe_, GST_STATE_NULL); gst_object_unref(right_pipe_); }
    if (left_sink_)  { gst_object_unref(left_sink_); }
    if (right_sink_) { gst_object_unref(right_sink_); }
}

std::string StereoSyncNode::make_pipeline_desc(int sensor_id) {
    std::ostringstream ss;
    ss
      << "nvarguscamerasrc sensor-id=" << sensor_id
      << " exposuretimerange=\"" << exposure_min_ << " " << exposure_max_ << "\""
      << " gainrange=\"" << gain_min_ << " " << gain_max_ << "\""
      << " ispdigitalgainrange=\"1 1\""
      << " wbmode=" << wbmode_
      << " ! video/x-raw(memory:NVMM),width=" << width_ << ",height=" << height_ << ",framerate=" << fps_ << "/1"
      << " ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR"
      << " ! appsink name=" << (sensor_id == sensor_left_ ? "appsink_left" : "appsink_right")
      << " max-buffers=1 drop=true sync=false";
    return ss.str();
}

void StereoSyncNode::setup_appsink(GstAppSink* sink) {
    gst_app_sink_set_emit_signals(sink, false); // When a new data sample arrives, appsink does not automatically emit the new-sample signal.
    gst_app_sink_set_drop(sink, true);  // When the buffer is full, if new data arrives, it will discard the old data
    gst_app_sink_set_max_buffers(sink, 1); // Set the maximum number of buffers to 1
} 

rclcpp::Time StereoSyncNode::stamp_from_gst(GstBuffer* buf) {
    const gint64 pts_ns = GST_BUFFER_PTS(buf); // get gst time stemp (ns)
    if (pts_ns == GST_CLOCK_TIME_NONE) { // if it invalid
      auto now = std::chrono::steady_clock::now().time_since_epoch();
      return rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    }
    return rclcpp::Time(static_cast<uint64_t>(pts_ns));
}

void StereoSyncNode::captureLoop(GstAppSink* sink, std::deque<Frame>& buf, const char* /*tag*/) {
    const guint64 timeout_ns = 5'000'000; // 5ms
    while (running_) {
      GstSample* sample = gst_app_sink_try_pull_sample(sink, timeout_ns); // get data
      if (!sample) { continue; }
  
      // Get the actual buffer from sample
      GstBuffer* buffer = gst_sample_get_buffer(sample);
      GstMapInfo map;
      if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) { // Map the buffer's memory into user space
        gst_sample_unref(sample); // Release sample
        continue;
      }
  
      // conver to gary image
      cv::Mat img(height_, width_, CV_8UC3, const_cast<guint8*>(map.data));
      cv::Mat gray_img;
      cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
  
      rclcpp::Time ts = stamp_from_gst(buffer); // Extract timestamp
  
      gst_buffer_unmap(buffer, &map); // Unmap memory
      gst_sample_unref(sample); // Release the sample reference
  
      {
        std::lock_guard<std::mutex> lk(mtx_);
        buf.push_back(Frame{ts, std::move(gray_img)});
        while (buf.size() > static_cast<size_t>(fps_ * 2)) buf.pop_front();
      }
      cv_.notify_all();
    }
}

void StereoSyncNode::syncLoop() {
    const double tol_s = static_cast<double>(tol_ms_) / 1000.0; // Maximum time difference
  
    while (running_) {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait_for(lk, std::chrono::milliseconds(2));  // Thread synchronization
  
      while (running_ && !buf_left_.empty() && !buf_right_.empty()) {
        // Use the earlier frame as a reference (key frame)
        bool left_earlier = (buf_left_.front().stamp <= buf_right_.front().stamp);
        auto& Qm = left_earlier ? buf_left_  : buf_right_; // main
        auto& Qs = left_earlier ? buf_right_ : buf_left_;
  
        const auto main_ts = Qm.front().stamp;
  
        size_t best = SIZE_MAX; double best_dt = 1e9;
        for (size_t i = 0; i < Qs.size(); ++i) {
          double dt = std::abs((Qs[i].stamp - main_ts).seconds());
          if (dt < best_dt) { best_dt = dt; best = i; }
          if (Qs[i].stamp > main_ts && dt > tol_s * 2) break;
        }
  
        if (best != SIZE_MAX && best_dt <= tol_s) {
          Frame A = std::move(Qm.front()); Qm.pop_front();
          Frame B = std::move(Qs[best]);   Qs.erase(Qs.begin() + best);
  
          const int64_t avg_ns = static_cast<int64_t>(
            (A.stamp.nanoseconds() + B.stamp.nanoseconds()) / 2);
          rclcpp::Time stamp_avg(static_cast<uint64_t>(avg_ns));
  
          // publish gray image
          auto msgL = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", A.img).toImageMsg();
          auto msgR = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", B.img).toImageMsg();
          msgL->header.stamp = stamp_avg;
          msgR->header.stamp = stamp_avg;
          msgL->header.frame_id = "stereo_left_optical_frame";
          msgR->header.frame_id = "stereo_right_optical_frame";
  
          lk.unlock();
          left_pub_->publish(*msgL);
          right_pub_->publish(*msgR);
          lk.lock();
        } else {
          Qm.pop_front();
        }
      }
    }
}

// DualCameraNode::DualCameraNode()
// : Node("dual_camera_node")
// {
    

//     cam1_ = std::make_unique<cv::VideoCapture>(
//         "nvarguscamerasrc sensor-id=1 sync=true ! video/x-raw(memory:NVMM), width=640, height=480, "
//         "format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! "
//         "video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! "
//         "video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);

//         cam0_ = std::make_unique<cv::VideoCapture>(
//             "nvarguscamerasrc sensor-id=0 sync=true ! video/x-raw(memory:NVMM), width=640, height=480, "
//             "format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! "
//             "video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! "
//             "video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
//     if (!cam0_->isOpened() || !cam1_->isOpened()) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to open cameras");
//         rclcpp::shutdown();
//     }

//     cam0_->set(cv::CAP_PROP_BUFFERSIZE, 2);
//     cam1_->set(cv::CAP_PROP_BUFFERSIZE, 2);
// }

// void DualCameraNode::init()
// {
//     it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
//     image_pub0_ = it_->advertise("camera0/image_raw", 1);
//     image_pub1_ = it_->advertise("camera1/image_raw", 1);

//     timer_ = create_wall_timer(
//         std::chrono::milliseconds(30),
//         std::bind(&DualCameraNode::timer_callback, this));

//     RCLCPP_INFO(get_logger(), "Dual camera node initialized");
// }

// void DualCameraNode::timer_callback()
// {
//     cv::Mat frame0, frame1;
//     rclcpp::Time stamp0, stamp1;

//     if(cam0_->grab()) {
//         stamp0 = now();
//         cam0_->retrieve(frame0);
//         if(!frame0.empty()) {
//             frame_queue0_.emplace_back(frame0, stamp0);
//             if(frame_queue0_.size() > frame_queue_size_) {
//                 frame_queue0_.pop_front();
//             }
//         }
//     }

//     if(cam1_->grab()) {
//         stamp1 = now();
//         cam1_->retrieve(frame1);
//         if(!frame1.empty()) {
//             frame_queue1_.emplace_back(frame1, stamp1);
//             if(frame_queue1_.size() > frame_queue_size_) {
//                 frame_queue1_.pop_front();
//             }
//         }
//     }

//     publish_synced_frames();
// }

// void DualCameraNode::publish_synced_frames()
// {
//     if(frame_queue0_.empty() || frame_queue1_.empty()) return;

//     auto& [latest_frame0, latest_stamp0] = frame_queue0_.back();
//     auto& [latest_frame1, latest_stamp1] = frame_queue1_.back();

//     double time_diff = (latest_stamp1 - latest_stamp0).seconds();

//     std::cout << time_diff << std::endl;
    
//     if(time_diff < max_sync_delay_) {
//         auto msg0 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", latest_frame0).toImageMsg();
//         auto msg1 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", latest_frame1).toImageMsg();

//         rclcpp::Time sync_stamp = std::max(latest_stamp0, latest_stamp1);
//         msg0->header.stamp = sync_stamp;
//         msg0->header.frame_id = "camera0";
//         msg1->header.stamp = sync_stamp;
//         msg1->header.frame_id = "camera1";

//         image_pub0_.publish(msg0);
//         image_pub1_.publish(msg1);

//         frame_queue0_.clear();
//         frame_queue1_.clear();
//     }
// }