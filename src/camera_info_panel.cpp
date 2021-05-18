#include "camera_info_panel.hpp"
#include <QButtonGroup>
#include <QCheckBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>

#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

namespace camera_info_plugins
{
constexpr int SLIDER_MAX = 1000;

CameraInfoPanel::CameraInfoPanel(QWidget* parent) : rviz::Panel(parent)
{
  reso_options.emplace_back("320x240", 320, 240);
  reso_options.emplace_back("640x480", 640, 480);
  reso_options.emplace_back("800x600", 800, 600);
  reso_options.emplace_back("1280x720", 1280, 720);

  QVBoxLayout* layout = new QVBoxLayout;

  {
    // ====================
    // 1st line
    QHBoxLayout* layout_1st = new QHBoxLayout;
    enable_check_ = new QCheckBox("Publish");
    layout_1st->addWidget(enable_check_);
    // topic name
    layout_1st->addWidget(new QLabel("TopicName:"));
    topic_edit_ = new QLineEdit("/camera/camera_info");
    layout_1st->addWidget(topic_edit_);
    // frame_id
    layout_1st->addWidget(new QLabel("FrameId:"));
    frame_edit_ = new QLineEdit("/camera_plugin");
    layout_1st->addWidget(frame_edit_);
    //
    layout->addLayout(layout_1st);
  }

  {
    // ====================
    // 2nd line
    QHBoxLayout* layout_3rd = new QHBoxLayout;
    layout_3rd->addWidget(new QLabel("OBSOLETE"));
    max1_edit_ = new QLineEdit("100");
    layout_3rd->addWidget(max1_edit_);
    layout_3rd->addWidget(new QLabel("OBSOLETE"));
    max2_edit_ = new QLineEdit("3.14");
    layout_3rd->addWidget(max2_edit_);
    layout->addLayout(layout_3rd);
  }

  {
    // ====================
    // 3rd line
    QHBoxLayout* layout_4th = new QHBoxLayout;
    for (int i = 0; i < reso_options.size(); i++) {
      const Resolution& reso = reso_options.at(i);
      radio_[i] = new QRadioButton(reso.label.c_str());
      layout_4th->addWidget(radio_[i]);
    }
    radio_[0]->setChecked(true);
    //
    layout->addLayout(layout_4th);
  }

  // ====================
  // 4th line
  {
    QHBoxLayout* layout_ith = new QHBoxLayout;
    // label
    focal_label_ = new QLabel("100");
    layout_ith->addWidget(focal_label_);
    // slider
    focal_slider_ = new QSlider(Qt::Orientation::Horizontal);
    focal_slider_->setRange(0, SLIDER_MAX);
    focal_slider_->setValue(SLIDER_MAX / 2);
    layout_ith->addWidget(focal_slider_);
    //
    layout->addLayout(layout_ith);
  }

  // ====================
  setLayout(layout);


  // ====================
  // Grouping radio button
  QButtonGroup* group = new QButtonGroup();
  for (int i = 0; i < 4; i++)
    group->addButton(radio_[i]);

  // timer callback
  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);
}


CameraInfoPanel::~CameraInfoPanel()
{
  if (camera_info_publisher_) camera_info_publisher_.shutdown();
}

void CameraInfoPanel::tick()
{
  const float tra_max = max1_edit_->text().toFloat();
  const float rot_max = max2_edit_->text().toFloat();

  std::vector<float> se3_value;
  std::vector<std::string> prefix = {"tx", "ty", "tz", "rx", "ry", "rz"};

  if (!ros::ok()) return;


  if (!enable_check_->isChecked()) {
    if (camera_info_publisher_) {
      camera_info_publisher_.shutdown();
      topic_edit_->setEnabled(true);
    }
    return;
  }

  if (enable_check_->isChecked()) {

    if (!camera_info_publisher_) {
      std::string topic_name = topic_edit_->text().toStdString();
      if (topic_name != "") {
        {
          camera_info_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>(topic_name, 10);
          topic_edit_->setEnabled(false);
        }
      } else {
        enable_check_->setChecked(false);
      }
    }

    if (camera_info_publisher_) {
      sensor_msgs::CameraInfo msg;
      camera_info_publisher_.publish(msg);
    }
  }
}

void CameraInfoPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Topic", topic_edit_->text());
  config.mapSetValue("max1", max1_edit_->text());
  config.mapSetValue("max2", max2_edit_->text());
}

void CameraInfoPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  QString tmp_text;
  if (config.mapGetString("Topic", &tmp_text))
    topic_edit_->setText(tmp_text);
  if (config.mapGetString("max1", &tmp_text))
    max1_edit_->setText(tmp_text);
  if (config.mapGetString("max2", &tmp_text))
    max2_edit_->setText(tmp_text);

  if (topic_edit_->text() == "")
    topic_edit_->setText("se3");
  if (max1_edit_->text() == "")
    max1_edit_->setText("1");
  if (max2_edit_->text() == "")
    max2_edit_->setText("3.14");
}
}  // namespace camera_info_plugins

PLUGINLIB_EXPORT_CLASS(camera_info_plugins::CameraInfoPanel, rviz::Panel)
