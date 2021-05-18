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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <vector>

namespace camera_info_plugins
{
constexpr int SLIDER_MAX = 50;

CameraInfoPanel::CameraInfoPanel(QWidget* parent) : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  QHBoxLayout* layout_1st = new QHBoxLayout;
  enable_check_ = new QCheckBox("Publish");
  layout_1st->addWidget(enable_check_);
  layout_1st->addWidget(new QLabel("Topic:"));
  topic_edit_ = new QLineEdit("se3");
  layout_1st->addWidget(topic_edit_);
  layout->addLayout(layout_1st);

  QHBoxLayout* layout_3rd = new QHBoxLayout;
  layout_3rd->addWidget(new QLabel("tra max:"));
  max1_edit_ = new QLineEdit("1");
  layout_3rd->addWidget(max1_edit_);
  layout_3rd->addWidget(new QLabel("rot max:"));
  max2_edit_ = new QLineEdit("3.14");
  layout_3rd->addWidget(max2_edit_);
  layout->addLayout(layout_3rd);

  QHBoxLayout* layout_4th = new QHBoxLayout;
  for (int i = 0; i < 4; i++) {
    radio_[i] = new QRadioButton(std::to_string(i).c_str());
    layout_4th->addWidget(radio_[i]);
  }
  radio_[0]->setChecked(true);
  layout->addLayout(layout_4th);

  for (int i = 0; i < 6; i++) {
    QHBoxLayout* layout_ith = new QHBoxLayout;

    se3_label_[i] = new QLabel("");
    layout_ith->addWidget(se3_label_[i]);

    se3_slider_[i] = new QSlider(Qt::Orientation::Horizontal);
    se3_slider_[i]->setRange(-SLIDER_MAX, SLIDER_MAX);
    layout_ith->addWidget(se3_slider_[i]);

    button_[i] = new QPushButton("clear");
    layout_ith->addWidget(button_[i]);
    connect(button_[i], &QPushButton::released, [=]() {
      se3_slider_[i]->setValue(0);
    });

    layout->addLayout(layout_ith);
  }

  setLayout(layout);

  // Grouping radio button
  QButtonGroup* group = new QButtonGroup();
  for (int i = 0; i < 4; i++)
    group->addButton(radio_[i]);

  // timer callback
  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);

  // Publisher
  mode_publisher_ = nh_.advertise<std_msgs::Int32>("mode", 10);
}


CameraInfoPanel::~CameraInfoPanel()
{
  if (se3_publisher_) se3_publisher_.shutdown();
  if (mode_publisher_) mode_publisher_.shutdown();
}

void CameraInfoPanel::tick()
{
  const float tra_max = max1_edit_->text().toFloat();
  const float rot_max = max2_edit_->text().toFloat();

  std::vector<float> se3_value;
  std::vector<std::string> prefix = {"tx", "ty", "tz", "rx", "ry", "rz"};

  for (int i = 0; i < 6; i++) {
    int tick = se3_slider_[i]->value();
    float value = static_cast<float>(tick) / static_cast<float>(SLIDER_MAX);

    if (i < 3)
      value *= tra_max;
    else
      value *= rot_max;

    se3_value.push_back(value);
    std::string str = prefix[i] + ": " + std::to_string(value);
    se3_label_[i]->setText(str.c_str());
  }

  if (!ros::ok()) return;


  if (!enable_check_->isChecked()) {
    if (se3_publisher_) {
      se3_publisher_.shutdown();
      topic_edit_->setEnabled(true);
    }
    return;
  }

  if (enable_check_->isChecked()) {

    if (!se3_publisher_) {
      std::string topic_name = topic_edit_->text().toStdString();
      if (topic_name != "") {
        {
          se3_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>(topic_name, 10);
          topic_edit_->setEnabled(false);
        }
      } else {
        enable_check_->setChecked(false);
      }
    }

    if (se3_publisher_) {
      std_msgs::Float32MultiArray msg;
      msg.data = se3_value;
      se3_publisher_.publish(msg);
    }
  }
  // publish mode
  {
    std_msgs::Int32 msg;
    for (int i = 0; i < 4; i++)
      if (radio_[i]->isChecked()) msg.data = i;
    mode_publisher_.publish(msg);
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
