#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <string>
#endif

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

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

  std::vector<Resolution> reso_options;

  QCheckBox* enable_check_;
  QLineEdit* topic_edit_;
  QLineEdit* frame_edit_;

  QLineEdit* max1_edit_;
  QLineEdit* max2_edit_;

  QSlider* focal_slider_;
  QLabel* focal_label_;

  QRadioButton* radio_[4];

  std::string pub_frame_;
};
}  // namespace camera_info_plugins