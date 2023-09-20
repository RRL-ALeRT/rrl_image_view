#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std;

bool removeSuffix(const std::string& input, const std::string& suffix, std::string& output)
{
  if (input.length() >= suffix.length() && input.substr(input.length() - suffix.length()) == suffix)
  {
    output = input.substr(0, input.length() - suffix.length());
    return true;
  }
  return false;
}

class ImageDisplayNode : public rclcpp::Node
{
public:
  ImageDisplayNode() : Node("image_display_node")
  {
    // Create a timer that calls the checkTopics function every second
    timer_ = this->create_wall_timer(0.1s, std::bind(&ImageDisplayNode::checkTopics, this));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    timer_.reset();
    mouse_topics.clear();
    try
    {
      auto cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      auto resized_image = resizeImage(cv_ptr->image, 1000, 600);
      cv::imshow("Resized Image", resized_image);
      cv::waitKey(1);

      cv::setMouseCallback("Resized Image", &ImageDisplayNode::mouseCallback, NULL);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
  
  static void mouseCallback(int event, int x, int y, int flags, void* userdata)
  {
    rclcpp::shutdown();
  }

  cv::Mat resizeImage(const cv::Mat& input_image, int target_width, int target_height)
  {
    cv::Mat output_image;
    double width_ratio = static_cast<double>(target_width) / input_image.cols;
    double height_ratio = static_cast<double>(target_height) / input_image.rows;
    double scale = std::min(width_ratio, height_ratio);
    cv::resize(input_image, output_image, cv::Size(), scale, scale);
    return output_image;
  }

  void checkTopics()
  {
    // Get a list of all available topics
    auto topic_names = this->get_topic_names_and_types();

    // Check if any of the topics have "/mouse" in their names
    for (const auto& topic : topic_names)
    {
      // Use find() to check if the key exists in the unordered_map
      auto it = mouse_topics.find(topic.first);

      if (it != mouse_topics.end()) continue; // topic found

      if (topic.first.find("/mouse") != std::string::npos)
      {
        std::string sub_topic;
        if (!removeSuffix(topic.first, "/mouse", sub_topic))
        {
          continue;
        }
        auto sub = this->create_subscription<geometry_msgs::msg::Point>(
          topic.first,
          1,
          [&, sub_topic](const geometry_msgs::msg::Point::SharedPtr msg) {
            (void) msg;
            final_image_topic_ = sub_topic + "/republished";
            RCLCPP_INFO(this->get_logger(), final_image_topic_.c_str());
          }
        );

        mouse_topics[topic.first] = sub;
      }
    }

    if (final_image_topic_ != "")
    {
      subscription_ = image_transport::create_subscription(this, final_image_topic_,
        std::bind(&ImageDisplayNode::imageCallback, this, std::placeholders::_1), "raw",
        rmw_qos_profile_sensor_data);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr> mouse_topics;

  image_transport::Subscriber subscription_;
  std::string final_image_topic_ = "";
  std::string prev_final_image_topic_ = "";
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageDisplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
