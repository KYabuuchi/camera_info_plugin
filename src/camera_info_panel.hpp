#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <string>
#endif

#include "interactive_marker.hpp"
#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <sensor_msgs/CameraInfo.h>

namespace camera_info_plugins
{
struct Resolution {
  Resolution(const std::string& label, int x, int y) : label(label), x(x), y(y) {}
  std::string label;
  int x, y;
};

class CameraInfoPanel : public rviz::Panel
{
  Q_OBJECT
public:
  CameraInfoPanel(QWidget* parent = 0);
  ~CameraInfoPanel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void tick();

public:
  // The ROS node handle.
  ros::NodeHandle nh_;
  // The ROS publisher for the command velocity.
  ros::Publisher camera_info_publisher_;
  ros::Publisher image_publisher_;

  std::vector<Resolution> reso_options;

  std::shared_ptr<InteMarker> interective_marker_ = nullptr;

  QButtonGroup* edit_group_;

  QCheckBox* enable_check_;
  QLineEdit* topic_edit_;
  QLineEdit* parent_frame_edit_;
  QLineEdit* camera_frame_edit_;

  QSlider* focal_slider_;
  QLabel* focal_label_;

  QLabel* msg_label_;

  QRadioButton* radio_[4];

  std::string pub_frame_;

private:
  sensor_msgs::CameraInfo makeCameraInfoMsg();
  std::string msgToText(const sensor_msgs::CameraInfo& msg);

  void setEditorEnable(bool enable);
};
}  // namespace camera_info_plugins