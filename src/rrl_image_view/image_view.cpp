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

#include <rrl_image_view/image_view.h>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

namespace rrl_image_view {

RRLImageView::RRLImageView()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , num_gridlines_(0)
  , rotate_state_(ROTATE_0)
{
  setObjectName("RRLImageView");
}

void RRLImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  updateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));

  ui_.save_as_image_push_button->setIcon(QIcon::fromTheme("document-save-as"));
  connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this, SLOT(saveImage()));

  // set topic name if passed in as argument
  const QStringList& argv = context.argv();
  if (!argv.empty()) {
    arg_topic_name = argv[0];
    selectTopic(arg_topic_name);
  }
  pub_topic_custom_ = false;

  ui_.image_frame->setOuterLayout(ui_.image_layout);

  QRegExp rx("([a-zA-Z/][a-zA-Z0-9_/]*)?"); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
  // ui_.publish_click_location_topic_line_edit->setValidator(new QRegExpValidator(rx, this));
  // connect(ui_.publish_click_location_check_box, SIGNAL(toggled(bool)), this, SLOT(onMousePublish(bool)));
  connect(ui_.image_frame, SIGNAL(mouseLeft(int, int)), this, SLOT(onMouseLeft(int, int)));
  // connect(ui_.publish_click_location_topic_line_edit, SIGNAL(editingFinished()), this, SLOT(onPubTopicChanged()));

  // connect(ui_.smooth_image_check_box, SIGNAL(toggled(bool)), ui_.image_frame, SLOT(onSmoothImageChanged(bool)));

  // connect(ui_.rotate_left_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateLeft()));
  // connect(ui_.rotate_right_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateRight()));

//   // Make sure we have enough space for "XXX °"
// #if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
//   // QFontMetrics::width(QChar) is deprecated starting from qt version 5.11.0
//   // https://doc.qt.io/qt-5/qfontmetrics.html#horizontalAdvance-1
//   ui_.rotate_label->setMinimumWidth(
//     ui_.rotate_label->fontMetrics().horizontalAdvance("XXX°")
//   );
// #else
//   ui_.rotate_label->setMinimumWidth(
//     ui_.rotate_label->fontMetrics().width("XXX°")
//   );
// #endif

  hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
  hide_toolbar_action_->setCheckable(true);
  ui_.image_frame->addAction(hide_toolbar_action_);
  connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this, SLOT(onHideToolbarChanged(bool)));
}

void RRLImageView::shutdownPlugin()
{
  subscriber_.shutdown();
  pub_mouse_left_.reset();
}

void RRLImageView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  (void)plugin_settings;
  QString topic = ui_.topics_combo_box->currentText();
  instance_settings.setValue("topic", topic);
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  instance_settings.setValue("toolbar_hidden", hide_toolbar_action_->isChecked());
  instance_settings.setValue("rotate", rotate_state_);
}

void RRLImageView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  (void)plugin_settings;
  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  QString topic = instance_settings.value("topic", "").toString();
  // don't overwrite topic name passed as command line argument
  if (!arg_topic_name.isEmpty())
  {
    arg_topic_name = "";
  }
  else
  {
    //qDebug("RRLImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
    selectTopic(topic);
  }

  bool publish_click_location = instance_settings.value("publish_click_location", false).toBool();
  // ui_.publish_click_location_check_box->setChecked(publish_click_location);

  QString pub_topic = instance_settings.value("mouse_pub_topic", "").toString();
  // ui_.publish_click_location_topic_line_edit->setText(pub_topic);

  bool toolbar_hidden = instance_settings.value("toolbar_hidden", false).toBool();
  hide_toolbar_action_->setChecked(toolbar_hidden);

  bool smooth_image_checked = instance_settings.value("smooth_image", false).toBool();
  // ui_.smooth_image_check_box->setChecked(smooth_image_checked);

  rotate_state_ = static_cast<RotateState>(instance_settings.value("rotate", 0).toInt());
  if(rotate_state_ >= ROTATE_STATE_COUNT)
    rotate_state_ = ROTATE_0;
  syncRotateLabel();

  // int color_scheme = instance_settings.value("color_scheme", ui_.color_scheme_combo_box->currentIndex()).toInt();
  // ui_.color_scheme_combo_box->setCurrentIndex(color_scheme);
}

void RRLImageView::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  message_types.insert("sensor_msgs/msg/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");
  message_sub_types.insert("sensor_msgs/msg/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(node_);
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("RRLImageView::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  std::sort(topics.begin(), topics.end());
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);

    // Check if the topic contains "compressedDepth", and if so, skip it
    if (label.contains("compressedDepth") || label.contains("depth"))
    {
      continue; // Skip this topic and move to the next iteration
    }

    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

QSet<QString> RRLImageView::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
  std::map<std::string, std::vector<std::string>> topic_info = node_->get_topic_names_and_types();

  QSet<QString> all_topics;
  for (std::map<std::string, std::vector<std::string>>::iterator it = topic_info.begin(); it != topic_info.end(); ++it)
  {
    all_topics.insert(it->first.c_str());
  }

  QSet<QString> topics;
  for (std::map<std::string, std::vector<std::string>>::iterator it = topic_info.begin(); it != topic_info.end(); ++it)
  {
    for (std::vector<std::string>::const_iterator msg_type_it = it->second.begin(); msg_type_it != it->second.end(); ++msg_type_it)
    {
      if (message_types.contains(msg_type_it->c_str()))
      {
        QString topic = it->first.c_str();

        // add raw topic
        topics.insert(topic);
        //qDebug("RRLImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

        // add transport specific sub-topics
        for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
        {
          if (all_topics.contains(topic + "/" + *jt))
          {
            QString sub = topic + " " + *jt;
            topics.insert(sub);
            //qDebug("RRLImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
          }
        }
      }
      if (message_sub_types.contains(msg_type_it->c_str()))
      {
        QString topic = it->first.c_str();
        int index = topic.lastIndexOf("/");
        if (index != -1)
        {
          topic.replace(index, 1, " ");
          topics.insert(topic);
          //qDebug("RRLImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
        }
      }
    }
  }
  return topics;
}

void RRLImageView::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(topic));
    index = ui_.topics_combo_box->findText(topic);
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void RRLImageView::onTopicChanged(int index)
{
  conversion_mat_.release();

  subscriber_.shutdown();
  pub_mouse_left_.reset();

  // reset image on topic change
  ui_.image_frame->setImage(QImage());

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    const image_transport::TransportHints hints(node_.get(), transport.toStdString());
    try {
      auto subscription_options = rclcpp::SubscriptionOptions();
      // TODO(jacobperron): Enable once ROS CLI args are supported https://github.com/ros-visualization/rqt/issues/262
      // subscription_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
      std::string image_topic = topic.toStdString();
      subscriber_ = image_transport::create_subscription(
        node_.get(),
        image_topic,
        std::bind(&RRLImageView::callbackImage, this, std::placeholders::_1),
        // hints.getTransport(),
       "ffmpeg",
        rmw_qos_profile_sensor_data,
        subscription_options);
      republisher_ = image_transport::create_publisher(
        node_.get(),
        image_topic + "/republished",
        rmw_qos_profile_sensor_data);
      bb_subscriber_ = node_->create_subscription<world_info_msgs::msg::BoundingBoxArray>(
        image_topic + "/bb",
        1,
        std::bind(&RRLImageView::callbackBoundingBox, this, std::placeholders::_1));
      bp_subscriber_ = node_->create_subscription<world_info_msgs::msg::BoundingPolygonArray>(
        image_topic + "/bp",
        1,
        std::bind(&RRLImageView::callbackBoundingPolygon, this, std::placeholders::_1));
      kp_subscriber_ = node_->create_subscription<world_info_msgs::msg::KeypointsArray>(
        image_topic + "/kp",
        1,
        std::bind(&RRLImageView::callbackKeypoints, this, std::placeholders::_1));
      pub_mouse_left_ = node_->create_publisher<geometry_msgs::msg::Point>(image_topic + "/mouse", 1000);
      qDebug("RRLImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    } catch (const rclcpp::exceptions::RCLError& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed."), (static_cast<std::string>(e.what()) + "\nThis error occurs mostly when same topic is selected in another rqt image viewer, and by default image viewer believes it's of the type sensor_msgs/msg/Image. Deselect those in the image viewer.").c_str());
    }
  }
}

void RRLImageView::onZoom1(bool checked)
{
  if (checked)
  {
    if (ui_.image_frame->getImage().isNull())
    {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(ui_.image_frame->getImage().size());
  } else {
    ui_.image_frame->setInnerFrameMinimumSize(QSize(320, 240));
    ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(320, 240));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void RRLImageView::onDynamicRange(bool checked)
{
  // ui_.max_range_double_spin_box->setEnabled(!checked);
}

void RRLImageView::updateNumGridlines()
{
  // num_gridlines_ = ui_.num_gridlines_spin_box->value();
}

void RRLImageView::saveImage()
{
  // take a snapshot before asking for the filename
  QImage img = ui_.image_frame->getImageCopy();

  QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png", tr("Image (*.bmp *.jpg *.png *.tiff)"));
  if (file_name.isEmpty())
  {
    return;
  }

  img.save(file_name);
}

void RRLImageView::onMouseLeft(int x, int y)
{
  if(!ui_.image_frame->getImage().isNull())
  {
    geometry_msgs::msg::Point clickCanvasLocation;
    // Publish click location in pixel coordinates
    clickCanvasLocation.x = round((double)x/(double)ui_.image_frame->width()*(double)ui_.image_frame->getImage().width());
    clickCanvasLocation.y = round((double)y/(double)ui_.image_frame->height()*(double)ui_.image_frame->getImage().height());
    clickCanvasLocation.z = 0;

    geometry_msgs::msg::Point clickLocation = clickCanvasLocation;

    // switch(rotate_state_)
    // {
    //   case ROTATE_90:
    //     clickLocation.x = clickCanvasLocation.y;
    //     clickLocation.y = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
    //     break;
    //   case ROTATE_180:
    //     clickLocation.x = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
    //     clickLocation.y = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
    //     break;
    //   case ROTATE_270:
    //     clickLocation.x = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
    //     clickLocation.y = clickCanvasLocation.x;
    //     break;
    //   default:
    //     break;
    // }

    pub_mouse_left_->publish(clickLocation);
  }
}

void RRLImageView::onHideToolbarChanged(bool hide)
{
  // ui_.toolbar_widget->setVisible(!hide);
}

void RRLImageView::onRotateLeft()
{
  int m = rotate_state_ - 1;
  if(m < 0)
    m = ROTATE_STATE_COUNT-1;

  rotate_state_ = static_cast<RotateState>(m);
  syncRotateLabel();
}

void RRLImageView::onRotateRight()
{
  rotate_state_ = static_cast<RotateState>((rotate_state_ + 1) % ROTATE_STATE_COUNT);
  syncRotateLabel();
}

void RRLImageView::syncRotateLabel()
{
  // switch(rotate_state_)
  // {
  //   default:
  //   case ROTATE_0:   ui_.rotate_label->setText("0°"); break;
  //   case ROTATE_90:  ui_.rotate_label->setText("90°"); break;
  //   case ROTATE_180: ui_.rotate_label->setText("180°"); break;
  //   case ROTATE_270: ui_.rotate_label->setText("270°"); break;
  // }
}

void RRLImageView::invertPixels(int x, int y)
{
  // Could do 255-conversion_mat_.at<cv::Vec3b>(cv::Point(x,y))[i], but that doesn't work well on gray
  cv::Vec3b & pixel = conversion_mat_.at<cv::Vec3b>(cv::Point(x, y));
  if (pixel[0] + pixel[1] + pixel[2] > 3 * 127)
    pixel = cv::Vec3b(0,0,0);
  else
    pixel = cv::Vec3b(255,255,255);
}

QList<int> RRLImageView::getGridIndices(int size) const
{
  QList<int> indices;

  // the spacing between adjacent grid lines
  float grid_width = 1.0f * size / (num_gridlines_ + 1);

  // select grid line(s) closest to the center
  float index;
  if (num_gridlines_ % 2)  // odd
  {
    indices.append(size / 2);
    // make the center line 2px wide in case of an even resolution
    if (size % 2 == 0)  // even
      indices.append(size / 2 - 1);
    index = 1.0f * (size - 1) / 2;
  }
  else  // even
  {
    index = grid_width * (num_gridlines_ / 2);
    // one grid line before the center
    indices.append(round(index));
    // one grid line after the center
    indices.append(size - 1 - round(index));
  }

  // add additional grid lines from the center to the border of the image
  int lines = (num_gridlines_ - 1) / 2;
  while (lines > 0)
  {
    index -= grid_width;
    indices.append(round(index));
    indices.append(size - 1 - round(index));
    lines--;
  }

  return indices;
}

void RRLImageView::overlayGrid()
{
  // vertical gridlines
  QList<int> columns = getGridIndices(conversion_mat_.cols);
  for (QList<int>::const_iterator x = columns.begin(); x != columns.end(); ++x)
  {
    for (int y = 0; y < conversion_mat_.rows; ++y)
    {
      invertPixels(*x, y);
    }
  }

  // horizontal gridlines
  QList<int> rows = getGridIndices(conversion_mat_.rows);
  for (QList<int>::const_iterator y = rows.begin(); y != rows.end(); ++y)
  {
    for (int x = 0; x < conversion_mat_.cols; ++x)
    {
      invertPixels(x, *y);
    }
  }
}

void RRLImageView::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;

    if (num_gridlines_ > 0)
      overlayGrid();
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        // double max = ui_.max_range_double_spin_box->value();
        double max = 2;
        if (msg->encoding == "16UC1") max *= 1000;
        // if (ui_.dynamic_range_check_box->isChecked())
        // {
        //   // dynamically adjust range based on min/max in image
        //   cv::minMaxLoc(cv_ptr->image, &min, &max);
        //   if (min == max) {
        //     // completely homogeneous images are displayed in gray
        //     min = 0;
        //     max = 2;
        //   }
        // }
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning("RRLImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        ui_.image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("RRLImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      ui_.image_frame->setImage(QImage());
      return;
    }
  }

  // Handle rotation
  switch(rotate_state_)
  {
    case ROTATE_90:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_, tmp);
      cv::flip(tmp, conversion_mat_, 1);
      break;
    }
    case ROTATE_180:
    {
      cv::Mat tmp;
      cv::flip(conversion_mat_, tmp, -1);
      conversion_mat_ = tmp;
      break;
    }
    case ROTATE_270:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_, tmp);
      cv::flip(tmp, conversion_mat_, 0);
      break;
    }
    default:
      break;
  }

  // Draw bounding boxes on the 'conversion_mat_' using OpenCV
  for (const auto& bb_array : bounding_box_map)
  {
    for (const auto& bb : bb_array.second)
    {
      // Ignore if bb data is older than 2 seconds
      if (msg->header.stamp.sec > bb.time_second + 2) continue;

      // Define the bounding box coordinates
      cv::Rect boundingBox(bb.x - 0.5 * bb.width, bb.y - 0.5 * bb.height, bb.width, bb.height);

      // Draw the bounding box using OpenCV
      cv::rectangle(conversion_mat_, boundingBox, (255, 0, 0), 4);

      // Define the text position within the bounding box
      cv::Point textPosition(boundingBox.x + 5, boundingBox.y - 5);

      // Draw the text inside the bounding box using OpenCV
      cv::putText(conversion_mat_, bb.text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2);
    }
  }

  // Draw polygon on the 'conversion_mat_' using OpenCV
  for (const auto& bp_array : bounding_polygon_map)
  {
    for (const auto& bp : bp_array.second)
    {
      // Ignore if bp data is older than 2 seconds
      if (msg->header.stamp.sec > bp.time_second + 2) continue;

      const cv::Point *pts = (const cv::Point*) cv::Mat(bp.contour).data;
      int npts = cv::Mat(bp.contour).rows;
      cv::polylines(conversion_mat_, &pts, &npts, 1, true, cv::Scalar(255, 0, 0));

      // Draw the text inside the polygon using OpenCV
      cv::putText(conversion_mat_, bp.text, bp.contour[0], cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2);
    }
  }

  // Draw keypoinys on the 'conversion_mat_' using OpenCV
  for (const auto& kp_array : keypoints_map)
  {
    for (const auto& kp : kp_array.second)
    {
      // Ignore if kp data is older than 2 seconds
      if (msg->header.stamp.sec > kp.time_second + 2) continue;

      // Draw each point as a visible marker
      for (const cv::Point& point : kp.keypoints) {
        cv::drawMarker(conversion_mat_, point, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
      }
    }
  }

  republisher_.publish(cv_bridge::CvImage(msg->header, "rgb8", conversion_mat_).toImageMsg());

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);

  ui_.image_frame->setImage(image);

  if (!ui_.zoom_1_push_button->isEnabled())
  {
    ui_.zoom_1_push_button->setEnabled(true);
  }
  // Need to update the zoom 1 every new image in case the image aspect ratio changed,
  // though could check and see if the aspect ratio changed or not.
  onZoom1(ui_.zoom_1_push_button->isChecked());
}

void RRLImageView::callbackBoundingBox(const world_info_msgs::msg::BoundingBoxArray::SharedPtr msg)
{
  bounding_box_array.clear();
  for (auto& bb: msg->array)
  {
    BoundingBox bb_qt;
    if (bb.confidence > 0)
    {
      // Convert bb.confidence to a string with up to 2 decimal places
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << bb.confidence;
      bb_qt.text = oss.str() + ":" + bb.name;
    }
    else
    {
      bb_qt.text = bb.name;
    }
    bb_qt.x = bb.cx;
    bb_qt.y = bb.cy;
    bb_qt.width = bb.width;
    bb_qt.height = bb.height;
    bb_qt.time_second = msg->header.stamp.sec;
    bounding_box_array.push_back(bb_qt);
  }
  bounding_box_map[msg->type] = bounding_box_array;
}

void RRLImageView::callbackBoundingPolygon(const world_info_msgs::msg::BoundingPolygonArray::SharedPtr msg)
{
  bounding_polygon_array.clear();
  for (auto& bp: msg->array)
  {
    BoundingPolygon bp_qt;
    if (bp.confidence > 0)
    {
      // Convert bp.confidence to a string with up to 2 decimal places
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << bp.confidence;
      bp_qt.text = oss.str() + ":" + bp.name;
    }
    else
    {
      bp_qt.text = bp.name;
    }
    for (auto& bp : bp.array)
    {
      bp_qt.contour.push_back(cv::Point(bp.x, bp.y));
    }
    bp_qt.time_second = msg->header.stamp.sec;
    bounding_polygon_array.push_back(bp_qt);
  }
  bounding_polygon_map[msg->type] = bounding_polygon_array;
}

void RRLImageView::callbackKeypoints(const world_info_msgs::msg::KeypointsArray::SharedPtr msg)
{
  keypoints_array.clear();
  for (auto& kp: msg->array)
  {
    Keypoints kp_qt;
    for (auto& kp : kp.array)
    {
      kp_qt.keypoints.push_back(cv::Point(kp.x, kp.y));
    }
    kp_qt.time_second = msg->header.stamp.sec;
    keypoints_array.push_back(kp_qt);
  }
  keypoints_map[msg->type] = keypoints_array;
}

}

PLUGINLIB_EXPORT_CLASS(rrl_image_view::RRLImageView, rqt_gui_cpp::Plugin)
