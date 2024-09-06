#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <statistics_msgs/msg/metrics_message.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>

namespace orbbec_camera {
// namespace tools {

class TopicStatistics : public rclcpp::Node {
 public:
  // 带有 rclcpp::NodeOptions 的构造函数
  TopicStatistics(const rclcpp::NodeOptions & options)
    : Node("topic_statistics", options) {
    this->declare_parameter("front_color_image", "/front_camera/color/image_raw");
    this->declare_parameter("front_depth_image", "/front_camera/depth/image_raw");
    this->declare_parameter("front_left_ir_image", "/front_camera/left_ir/image_raw");
    this->declare_parameter("front_right_ir_image", "/front_camera/right_ir/image_raw");

    this->declare_parameter("left_color_image", "/left_camera/color/image_raw");
    this->declare_parameter("left_depth_image", "/left_camera/depth/image_raw");
    this->declare_parameter("left_left_ir_image", "/left_camera/left_ir/image_raw");
    this->declare_parameter("left_right_ir_image", "/left_camera/right_ir/image_raw");

    this->declare_parameter("right_color_image", "/right_camera/color/image_raw");
    this->declare_parameter("right_depth_image", "/right_camera/depth/image_raw");
    this->declare_parameter("right_left_ir_image", "/right_camera/left_ir/image_raw");
    this->declare_parameter("right_right_ir_image", "/right_camera/right_ir/image_raw");

    this->declare_parameter("rear_color_image", "/rear_camera/color/image_raw");
    this->declare_parameter("rear_depth_image", "/rear_camera/depth/image_raw");
    this->declare_parameter("rear_left_ir_image", "/rear_camera/left_ir/image_raw");
    this->declare_parameter("rear_right_ir_image", "/rear_camera/right_ir/image_raw");

    this->declare_parameter("statistics_topic", "/statistics");

    front_color_image_topic_ = this->get_parameter("front_color_image").as_string();
    front_depth_image_topic_ = this->get_parameter("front_depth_image").as_string();
    front_left_ir_image_topic_ = this->get_parameter("front_left_ir_image").as_string();
    front_right_ir_image_topic_ = this->get_parameter("front_right_ir_image").as_string();

    left_color_image_topic_ = this->get_parameter("left_color_image").as_string();
    left_depth_image_topic_ = this->get_parameter("left_depth_image").as_string();
    left_left_ir_image_topic_ = this->get_parameter("left_left_ir_image").as_string();
    left_right_ir_image_topic_ = this->get_parameter("left_right_ir_image").as_string();

    right_color_image_topic_ = this->get_parameter("right_color_image").as_string();
    right_depth_image_topic_ = this->get_parameter("right_depth_image").as_string();
    right_left_ir_image_topic_ = this->get_parameter("right_left_ir_image").as_string();
    right_right_ir_image_topic_ = this->get_parameter("right_right_ir_image").as_string();

    rear_color_image_topic_ = this->get_parameter("rear_color_image").as_string();
    rear_depth_image_topic_ = this->get_parameter("rear_depth_image").as_string();
    rear_left_ir_image_topic_ = this->get_parameter("rear_left_ir_image").as_string();
    rear_right_ir_image_topic_ = this->get_parameter("rear_right_ir_image").as_string();

    RCLCPP_INFO(get_logger(), "Front color image topic: %s", front_color_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Front depth image topic: %s", front_depth_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Front left IR image topic: %s", front_left_ir_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Front right IR image topic: %s", front_right_ir_image_topic_.c_str());

    RCLCPP_INFO(get_logger(), "Left color image topic: %s", left_color_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Left depth image topic: %s", left_depth_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Left left IR image topic: %s", left_left_ir_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Left right IR image topic: %s", left_right_ir_image_topic_.c_str());

    RCLCPP_INFO(get_logger(), "Right color image topic: %s", right_color_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Right depth image topic: %s", right_depth_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Right left IR image topic: %s", right_left_ir_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Right right IR image topic: %s", right_right_ir_image_topic_.c_str());

    RCLCPP_INFO(get_logger(), "Rear color image topic: %s", rear_color_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Rear depth image topic: %s", rear_depth_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Rear left IR image topic: %s", rear_left_ir_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Rear right IR image topic: %s", rear_right_ir_image_topic_.c_str());

    statistics_topic_ = this->get_parameter("statistics_topic").as_string();
    RCLCPP_INFO(get_logger(), "Statistics topic: %s", statistics_topic_.c_str());

    RCLCPP_INFO(get_logger(), "TopicStatistics starting up");
    initialize();
    initialize_csv();
  }

  void initialize() {
    // Create a subscriber to the image topic
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_topic = statistics_topic_;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(1000);

    front_color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        front_color_image_topic_, 10, std::bind(&TopicStatistics::front_color_image_callback, this, std::placeholders::_1), sub_opt);
    front_depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        front_depth_image_topic_, 10, std::bind(&TopicStatistics::front_depth_image_callback, this, std::placeholders::_1), sub_opt);
    front_left_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        front_left_ir_image_topic_, 10, std::bind(&TopicStatistics::front_left_ir_image_callback, this, std::placeholders::_1), sub_opt);
    front_right_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        front_right_ir_image_topic_, 10, std::bind(&TopicStatistics::front_right_ir_image_callback, this, std::placeholders::_1), sub_opt);

    left_color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        left_color_image_topic_, 10, std::bind(&TopicStatistics::left_color_image_callback, this, std::placeholders::_1), sub_opt);
    left_depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        left_depth_image_topic_, 10, std::bind(&TopicStatistics::left_depth_image_callback, this, std::placeholders::_1), sub_opt);
    left_left_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        left_left_ir_image_topic_, 10, std::bind(&TopicStatistics::left_left_ir_image_callback, this, std::placeholders::_1), sub_opt);
    left_right_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        left_right_ir_image_topic_, 10, std::bind(&TopicStatistics::left_right_ir_image_callback, this, std::placeholders::_1), sub_opt);

    right_color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        right_color_image_topic_, 10, std::bind(&TopicStatistics::right_color_image_callback, this, std::placeholders::_1), sub_opt);
    right_depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        right_depth_image_topic_, 10, std::bind(&TopicStatistics::right_depth_image_callback, this, std::placeholders::_1), sub_opt);
    right_left_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        right_left_ir_image_topic_, 10, std::bind(&TopicStatistics::right_left_ir_image_callback, this, std::placeholders::_1), sub_opt);
    right_right_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        right_right_ir_image_topic_, 10, std::bind(&TopicStatistics::right_right_ir_image_callback, this, std::placeholders::_1), sub_opt);

    rear_color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rear_color_image_topic_, 10, std::bind(&TopicStatistics::rear_color_image_callback, this, std::placeholders::_1), sub_opt);
    rear_depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rear_depth_image_topic_, 10, std::bind(&TopicStatistics::rear_depth_image_callback, this, std::placeholders::_1), sub_opt);
    rear_left_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rear_left_ir_image_topic_, 10, std::bind(&TopicStatistics::rear_left_ir_image_callback, this, std::placeholders::_1), sub_opt);
    rear_right_ir_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rear_right_ir_image_topic_, 10, std::bind(&TopicStatistics::rear_right_ir_image_callback, this, std::placeholders::_1), sub_opt);

    // Create a subscriber to the statistics topic
    statistics_sub_ = this->create_subscription<statistics_msgs::msg::MetricsMessage>(
        statistics_topic_, 10,
        std::bind(&TopicStatistics::statistics_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to statistics topic: %s", statistics_topic_.c_str());
  }

  void initialize_csv() {
    // Get the current working directory
    std::filesystem::path cwd = std::filesystem::current_path();
    csv_path_ = cwd.string() + "/statistics.csv";

    // Open the file in output mode, which will create/overwrite the file
    std::ofstream csv_file(csv_path_, std::ios::out);
    if (csv_file.is_open()) {
      // Write the header to the CSV file
      csv_file << "\"_time\",\"message_type\",\"min\",\"avg\",\"max\",\"sample_count\",\"std_dev\"\n";
      csv_file.close();
      csv_initialized_ = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open statistics.csv for writing");
    }
  }

  void write_statistics_to_csv(const statistics_msgs::msg::MetricsMessage& msg) {
    if (!csv_initialized_) {
      // If the CSV hasn't been initialized, initialize it
      initialize_csv();
    }

    std::ofstream csv_file(csv_path_, std::ios::app);
    if (csv_file.is_open()) {
      // Extract the timestamp from the MetricsMessage
      auto time_ns = rclcpp::Time(msg.window_stop);
      auto time_point = std::chrono::system_clock::from_time_t(time_ns.seconds());
      std::time_t time_t_value = std::chrono::system_clock::to_time_t(time_point);
      std::tm* tm_value = std::gmtime(&time_t_value);
      char time_str[20];
      std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_value);

      // Determine the message type (e.g., "age" or "period")
      std::string message_type;
      if (msg.metrics_source.find("_age") != std::string::npos) {
        message_type = "age";
      } else if (msg.metrics_source.find("_period") != std::string::npos) {
        message_type = "period";
      } else {
        message_type = "unknown";
      }

      if (message_type == "age") {
        return;  // Age statistics are not written to CSV
      }

      // Write the timestamp and message type to the CSV file
      csv_file << "\"" << time_str << "\",\"" << message_type << "\",";

      // Variables to store the min, avg, and max values
      double min = 0, avg = 0, max = 0;
      double sample_count = 0, std_dev = 0;

      // Iterate through the statistics and assign values based on the type
      for (const auto& statistic : msg.statistics) {
        switch (statistic.data_type) {
          case 1:  // avg
            avg = statistic.data;
            break;
          case 2:  // min
            min = statistic.data;
            break;
          case 3:  // max
            max = statistic.data;
            break;
          case 4:  // std_dev
            std_dev = statistic.data;
            break;
          case 5:  // sample_count
            sample_count = statistic.data;
            break;
          default:
            break;
        }
      }

      // Write the min, avg, and max values to the CSV file
      csv_file << min << " ms," << avg << " ms," << max << " ms, " << sample_count << " , "<< std_dev << "\n";
      csv_file.close();
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open statistics.csv for writing");
    }
  }

 private:
  void front_color_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    front_color_image_count_++;
    front_color_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "front_color_image_callback image %d, size: %zu bytes", front_color_image_count_,
                 msg->data.size());
  }
  void front_depth_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    front_depth_image_count_++;
    front_depth_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "front_depth_image_callback image %d, size: %zu bytes", front_depth_image_count_,
                 msg->data.size());
  }
  void front_left_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    front_left_ir_image_count_++;
    front_left_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "front_left_ir_image_callback image %d, size: %zu bytes", front_left_ir_image_count_,
                 msg->data.size());
  }
  void front_right_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    front_right_ir_image_count_++;
    front_right_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "front_right_ir_image_callback image %d, size: %zu bytes", front_right_ir_image_count_,
                 msg->data.size());
  }

  void left_color_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    left_color_image_count_++;
    left_color_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "left_color_image_callback image %d, size: %zu bytes", left_color_image_count_,
                 msg->data.size());
  }
  void left_depth_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    left_depth_image_count_++;
    left_depth_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "left_depth_image_callback image %d, size: %zu bytes", left_depth_image_count_,
                 msg->data.size());
  }
  void left_left_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    left_left_ir_image_count_++;
    left_left_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "left_left_ir_image_callback image %d, size: %zu bytes", left_left_ir_image_count_,
                 msg->data.size());
  }
  void left_right_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    left_right_ir_image_count_++;
    left_right_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "left_right_ir_image_callback image %d, size: %zu bytes", left_right_ir_image_count_,
                 msg->data.size());
  }

  void right_color_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    right_color_image_count_++;
    right_color_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "right_color_image_callback image %d, size: %zu bytes", right_color_image_count_,
                 msg->data.size());
  }
  void right_depth_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    right_depth_image_count_++;
    right_depth_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "right_depth_image_callback image %d, size: %zu bytes", right_depth_image_count_,
                 msg->data.size());
  }
  void right_left_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    right_left_ir_image_count_++;
    right_left_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "right_left_ir_image_callback image %d, size: %zu bytes", right_left_ir_image_count_,
                 msg->data.size());
  }
  void right_right_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    right_right_ir_image_count_++;
    right_right_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "right_right_ir_image_callback image %d, size: %zu bytes", right_right_ir_image_count_,
                 msg->data.size());
  }

  void rear_color_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    rear_color_image_count_++;
    rear_color_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "rear_color_image_callback image %d, size: %zu bytes", rear_color_image_count_,
                 msg->data.size());
  }
  void rear_depth_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    rear_depth_image_count_++;
    rear_depth_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "rear_depth_image_callback image %d, size: %zu bytes", rear_depth_image_count_,
                 msg->data.size());
  }
  void rear_left_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    rear_left_ir_image_count_++;
    rear_left_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "rear_left_ir_image_callback image %d, size: %zu bytes", rear_left_ir_image_count_,
                 msg->data.size());
  }
  void rear_right_ir_image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    rear_right_ir_image_count_++;
    rear_right_ir_image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "rear_right_ir_image_callback image %d, size: %zu bytes", rear_right_ir_image_count_,
                 msg->data.size());
  }

  void statistics_callback(const statistics_msgs::msg::MetricsMessage::UniquePtr msg) {
    // RCLCPP_INFO(get_logger(), "Statistics received:\n%s", metrics_message_to_string(*msg).c_str());
    write_statistics_to_csv(*msg);
  }

  std::string metrics_message_to_string(const statistics_msgs::msg::MetricsMessage& msg) {
    std::stringstream ss;
    std::string message_type;
    if (msg.metrics_source.find("_age") != std::string::npos) {
      message_type = "age";
    } else if (msg.metrics_source.find("_period") != std::string::npos) {
      message_type = "period";
    } else {
      message_type = "unknown";
    }
    ss << "message_type: " << message_type << "\n";

    ss << "Metric name: " << msg.metrics_source << " source: " << msg.measurement_source_name
       << " unit: " << msg.unit;
    ss << "\nWindow start: " << msg.window_start.nanosec << " end: " << msg.window_stop.nanosec;

    for (const auto& statistic : msg.statistics) {
      ss << "\n"
         << statistic_type_to_string(statistic.data_type) << ": " << std::to_string(statistic.data);
    }

    return ss.str();
  }

  std::string statistic_type_to_string(int8_t type) {
    switch (type) {
      case 1:
        return "avg";
      case 2:
        return "min";
      case 3:
        return "max";
      case 4:
        return "std_dev";
      case 5:
        return "sample_count";
      default:
        return "unknown";
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_left_ir_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_right_ir_image_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_left_ir_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_right_ir_image_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_left_ir_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_right_ir_image_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_left_ir_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_right_ir_image_sub_;

  rclcpp::Subscription<statistics_msgs::msg::MetricsMessage>::SharedPtr statistics_sub_;

  std::string front_color_image_topic_;
  std::string front_depth_image_topic_;
  std::string front_left_ir_image_topic_;
  std::string front_right_ir_image_topic_;

  std::string left_color_image_topic_;
  std::string left_depth_image_topic_;
  std::string left_left_ir_image_topic_;
  std::string left_right_ir_image_topic_;

  std::string right_color_image_topic_;
  std::string right_depth_image_topic_;
  std::string right_left_ir_image_topic_;
  std::string right_right_ir_image_topic_;

  std::string rear_color_image_topic_;
  std::string rear_depth_image_topic_;
  std::string rear_left_ir_image_topic_;
  std::string rear_right_ir_image_topic_;

  int front_color_image_count_ = 0;
  size_t front_color_image_size_ = 0;
  int front_depth_image_count_ = 0;
  size_t front_depth_image_size_ = 0;
  int front_left_ir_image_count_ = 0;
  size_t front_left_ir_image_size_ = 0;
  int front_right_ir_image_count_ = 0;
  size_t front_right_ir_image_size_ = 0;

  int left_color_image_count_ = 0;
  size_t left_color_image_size_ = 0;
  int left_depth_image_count_ = 0;
  size_t left_depth_image_size_ = 0;
  int left_left_ir_image_count_ = 0;
  size_t left_left_ir_image_size_ = 0;
  int left_right_ir_image_count_ = 0;
  size_t left_right_ir_image_size_ = 0;

  int right_color_image_count_ = 0;
  size_t right_color_image_size_ = 0;
  int right_depth_image_count_ = 0;
  size_t right_depth_image_size_ = 0;
  int right_left_ir_image_count_ = 0;
  size_t right_left_ir_image_size_ = 0;
  int right_right_ir_image_count_ = 0;
  size_t right_right_ir_image_size_ = 0;

  int rear_color_image_count_ = 0;
  size_t rear_color_image_size_ = 0;
  int rear_depth_image_count_ = 0;
  size_t rear_depth_image_size_ = 0;
  int rear_left_ir_image_count_ = 0;
  size_t rear_left_ir_image_size_ = 0;
  int rear_right_ir_image_count_ = 0;
  size_t rear_right_ir_image_size_ = 0;

  std::string statistics_topic_;
  std::string csv_path_;
  bool csv_initialized_ = false;
};

// }  // namespace tools
}  // namespace orbbec_camera
