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
    QHBoxLayout* layout_h = new QHBoxLayout;
    enable_check_ = new QCheckBox("Publish");
    layout_h->addWidget(enable_check_);
    // topic name
    layout_h->addWidget(new QLabel("topic:"));
    topic_edit_ = new QLineEdit("/camera_plugin/camera_info");
    layout_h->addWidget(topic_edit_);
    // frame_id
    layout_h->addWidget(new QLabel("frame_id:"));
    frame_edit_ = new QLineEdit("/camera_plugin");
    layout_h->addWidget(frame_edit_);
    //
    layout->addLayout(layout_h);
  }

  {
    // ====================
    QHBoxLayout* layout_h = new QHBoxLayout;
    for (int i = 0; i < reso_options.size(); i++) {
      const Resolution& reso = reso_options.at(i);
      radio_[i] = new QRadioButton(reso.label.c_str());
      layout_h->addWidget(radio_[i]);
    }
    radio_[0]->setChecked(true);
    //
    layout->addLayout(layout_h);

    // ====================
    // Grouping radio button
    QButtonGroup* group = new QButtonGroup();
    for (int i = 0; i < 4; i++)
      group->addButton(radio_[i]);
  }

  {
    // ====================
    QHBoxLayout* layout_h = new QHBoxLayout;
    // label
    focal_label_ = new QLabel("100");
    layout_h->addWidget(focal_label_);
    // slider
    focal_slider_ = new QSlider(Qt::Orientation::Horizontal);
    focal_slider_->setRange(0, SLIDER_MAX);
    focal_slider_->setValue(SLIDER_MAX / 2);
    layout_h->addWidget(focal_slider_);
    //
    layout->addLayout(layout_h);
  }

  {
    // ====================
    QHBoxLayout* layout_h = new QHBoxLayout;
    // label
    msg_label_ = new QLabel("");
    layout_h->addWidget(msg_label_);
    layout->addLayout(layout_h);
  }


  // ====================
  setLayout(layout);


  // timer callback
  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);  // 100[Hz]
}


CameraInfoPanel::~CameraInfoPanel()
{
  if (camera_info_publisher_) camera_info_publisher_.shutdown();
}

void CameraInfoPanel::tick()
{
  {
    std::string str = std::to_string(focal_slider_->value());
    focal_label_->setText(str.c_str());
  }


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
          camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_name, 10);
          topic_edit_->setEnabled(false);
        }
      } else {
        enable_check_->setChecked(false);
      }
    }
  }

  if (camera_info_publisher_) {
    auto msg = makeCameraInfoMsg();
    camera_info_publisher_.publish(msg);
    msg_label_->setText(msgToText(msg).c_str());
  }
}

void CameraInfoPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Topic", topic_edit_->text());
}

void CameraInfoPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  QString tmp_text;
  if (config.mapGetString("Topic", &tmp_text)) topic_edit_->setText(tmp_text);
  if (topic_edit_->text() == "") topic_edit_->setText("/camera_plugin/camera_info");
}

sensor_msgs::CameraInfo CameraInfoPanel::makeCameraInfoMsg()
{
  const float F = static_cast<float>(focal_slider_->value());
  float rx, ry;
  for (int i = 0; i < 4; i++) {
    if (!radio_[i]->isChecked()) continue;
    rx = reso_options.at(i).x;
    ry = reso_options.at(i).y;
  }

  const double FX = F;
  const double FY = F;
  const double RX = rx;
  const double RY = ry;
  const double CX = RX / 2;
  const double CY = RY / 2;

  static int seq = 0;

  sensor_msgs::CameraInfo msg;
  msg.header.frame_id = frame_edit_->text().toStdString();
  msg.header.stamp = ros::Time::now();
  msg.header.seq = seq++;

  msg.binning_x = 0;
  msg.binning_y = 0;
  msg.height = RY;
  msg.width = RX;
  msg.roi = sensor_msgs::RegionOfInterest();

  msg.distortion_model = "plumb_bob";
  msg.D = std::vector<double>{0, 0, 0, 0, 0};

  msg.K = boost::array<double, 9>{FX, 0, CX, 0, FY, CY, 0, 0, 1};
  msg.P = boost::array<double, 12>{FX, 0, CX, 0, 0, FY, CY, 0, 0, 0, 1, 0};

  return msg;
}

std::string CameraInfoPanel::msgToText(const sensor_msgs::CameraInfo& msg)
{
  std::stringstream ss;
  ss << "stamp: " << msg.header.stamp << "\n";
  ss << "seq: " << msg.header.seq << "\n";
  ss << "frame_id: " << msg.header.frame_id << "\n";
  return ss.str();
}

}  // namespace camera_info_plugins


PLUGINLIB_EXPORT_CLASS(camera_info_plugins::CameraInfoPanel, rviz::Panel)
