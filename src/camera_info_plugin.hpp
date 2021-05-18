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

namespace se3test
{
class Se3Panel : public rviz::Panel
{
  Q_OBJECT
public:
  Se3Panel(QWidget* parent = 0);
  ~Se3Panel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void tick();

public:
  // The ROS node handle.
  ros::NodeHandle nh_;
  // The ROS publisher for the command velocity.
  ros::Publisher se3_publisher_;
  ros::Publisher mode_publisher_;

  QCheckBox* enable_check_;
  QLineEdit* topic_edit_;

  QLineEdit* max1_edit_;
  QLineEdit* max2_edit_;

  QSlider* se3_slider_[6];
  QLabel* se3_label_[6];
  QPushButton* button_[6];

  QRadioButton* radio_[4];

  std::string pub_frame_;
};
}  // namespace se3test