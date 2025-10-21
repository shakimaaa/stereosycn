#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <deque>
#include <atomic>
#include <condition_variable>
#include <sstream>

struct Frame {
    rclcpp::Time stamp;
    cv::Mat img;  // gray MONO8
};

class StereoSyncNode : public rclcpp::Node {
public:
    StereoSyncNode();
    ~StereoSyncNode() override;
private:
    std::string make_pipeline_desc(int sensor_id);
    static void setup_appsink(GstAppSink* sink);
    static rclcpp::Time stamp_from_gst(GstBuffer* buf);

    void captureLoop(GstAppSink* sink, std::deque<Frame>& buf, const char* tag);
    void syncLoop();

    int width_{1280}, height_{720}, fps_{30}, tol_ms_{8};
    int sensor_left_{0}, sensor_right_{1};
    int exposure_min_{100000}, exposure_max_{10000000}, wbmode_{1};
    double gain_min_{1.0}, gain_max_{16.0};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

    GstElement *left_pipe_{nullptr}, *right_pipe_{nullptr};
    GstAppSink *left_sink_{nullptr}, *right_sink_{nullptr};

    std::thread cap_left_th_, cap_right_th_, sync_th_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<Frame> buf_left_, buf_right_;
    std::atomic<bool> running_{true};
};


// #include <rclcpp/rclcpp.hpp>
// #include <image_transport/image_transport.hpp>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <deque>
// #include <utility>

// class DualCameraNode : public rclcpp::Node
// {
// public:
//     DualCameraNode();
//     void init();

// private:
//     void timer_callback();
//     void publish_synced_frames();

//     std::unique_ptr<cv::VideoCapture> cam0_;
//     std::unique_ptr<cv::VideoCapture> cam1_;
    
//     std::shared_ptr<image_transport::ImageTransport> it_;
//     image_transport::Publisher image_pub0_;
//     image_transport::Publisher image_pub1_;
    
//     rclcpp::TimerBase::SharedPtr timer_;
    
//     std::deque<std::pair<cv::Mat, rclcpp::Time>> frame_queue0_;
//     std::deque<std::pair<cv::Mat, rclcpp::Time>> frame_queue1_;
    
//     const size_t frame_queue_size_ = 5;
//     const double max_sync_delay_ = 0.02; // 20ms
// };