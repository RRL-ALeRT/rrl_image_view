/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rrl_image_view__RRLImageView_H
#define rrl_image_view__RRLImageView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <world_info_msgs/msg/bounding_box_array.hpp>
#include <world_info_msgs/msg/bounding_polygon_array.hpp>
#include <world_info_msgs/msg/keypoints_array.hpp>

#include <opencv2/core/core.hpp>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>

#include <vector>

#include <QMainWindow>
#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>
#include <QApplication>
#include <QDesktopWidget>

#include <geometry_msgs/msg/point.hpp>

namespace rrl_image_view {

class EnlargedImageWindow : public QMainWindow
{
  Q_OBJECT

public:
  EnlargedImageWindow(QWidget *parent = nullptr)
      : QMainWindow(parent), label_(new QLabel(this))
  {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label_);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);

    setCentralWidget(centralWidget);
    setWindowTitle("Enlarged Image");

    // Set initial size
    resize(700, 700);

    // Move the window to the right edge of the screen
    moveToRightEdge();
  }

  void updatePixmap(const QPixmap &pixmap)
  {
    label_->setPixmap(pixmap);

    // Calculate the aspect ratio and adjust the width based on a height of 700
    int newHeight = 700;
    int newWidth = static_cast<int>(newHeight * pixmap.width() / static_cast<double>(pixmap.height()));
    resize(newWidth, newHeight);
  }

protected:
  void mousePressEvent(QMouseEvent *event) override
  {
    QMainWindow::mousePressEvent(event);
    close(); // Close the window on tap
  }

private:
  QLabel *label_;

  void moveToRightEdge()
  {
    QRect screenGeometry = QApplication::desktop()->availableGeometry(this);
    int x = screenGeometry.width() - width();
    int y = (screenGeometry.height() - height()) / 2; // Center vertically
    move(x, y);
  }
};

class RRLImageView
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  RRLImageView();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  virtual void updateTopicList();

protected:

  virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);

  virtual void onZoom1(bool checked);

  virtual void onDynamicRange(bool checked);

  virtual void saveImage();

  virtual void updateNumGridlines();

  virtual void onMouseLeft(int x, int y);

  virtual void onHideToolbarChanged(bool hide);

  virtual void onRotateLeft();
  virtual void onRotateRight();

protected:

  virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  
  virtual void callbackBoundingBox(const world_info_msgs::msg::BoundingBoxArray::SharedPtr msg);

  virtual void callbackBoundingPolygon(const world_info_msgs::msg::BoundingPolygonArray::SharedPtr msg);

  virtual void callbackKeypoints(const world_info_msgs::msg::KeypointsArray::SharedPtr msg);

  virtual void invertPixels(int x, int y);

  QList<int> getGridIndices(int size) const;

  virtual void overlayGrid();

  Ui::RRLImageViewWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;
  
  rclcpp::Subscription<world_info_msgs::msg::BoundingBoxArray>::SharedPtr bb_subscriber_;
  rclcpp::Subscription<world_info_msgs::msg::BoundingPolygonArray>::SharedPtr bp_subscriber_;
  rclcpp::Subscription<world_info_msgs::msg::KeypointsArray>::SharedPtr kp_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_mouse_left_;

  struct BoundingBox
  {
    std::string text;
    float x;
    float y;
    float width;
    float height;
    int time_second;
  };
  std::vector<BoundingBox> bounding_box_array;
  std::unordered_map<std::string, std::vector<BoundingBox>> bounding_box_map;

  struct BoundingPolygon
  {
    std::string text;
    std::vector<cv::Point> contour;
    int time_second;
  };
  std::vector<BoundingPolygon> bounding_polygon_array;
  std::unordered_map<std::string, std::vector<BoundingPolygon>> bounding_polygon_map;

  struct Keypoints
  {
    std::vector<cv::Point> keypoints;
    int time_second;
  };
  std::vector<Keypoints> keypoints_array;
  std::unordered_map<std::string, std::vector<Keypoints>> keypoints_map;

  cv::Mat conversion_mat_;

private:

  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  void syncRotateLabel();

  QString arg_topic_name;

  bool pub_topic_custom_;

  QAction* hide_toolbar_action_;

  int num_gridlines_;

  RotateState rotate_state_;

  EnlargedImageWindow *enlargedWindow_; // Pointer to the enlarged image window
};

}

#endif // rrl_image_view__RRLImageView_H
