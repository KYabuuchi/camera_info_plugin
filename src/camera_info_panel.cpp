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
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
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
    topic_edit_ = new QLineEdit("/camera_plugin");
    layout_h->addWidget(topic_edit_);
    // parent frame_id
    layout_h->addWidget(new QLabel("parent_frame:"));
    parent_frame_edit_ = new QLineEdit("world");
    layout_h->addWidget(parent_frame_edit_);
    // camera frame_id
    layout_h->addWidget(new QLabel("camera_frame:"));
    camera_frame_edit_ = new QLineEdit("/camera_plugin");
    layout_h->addWidget(camera_frame_edit_);
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
  if (image_publisher_) image_publisher_.shutdown();
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
      image_publisher_.shutdown();
      interective_marker_ = nullptr;
      setEditorEnable(true);
    }
    return;
  }

  if (enable_check_->isChecked()) {

    if (!camera_info_publisher_) {
      std::string topic_name = topic_edit_->text().toStdString();

      if (topic_name != "") {
        {
          camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_name + "/camera_info", 10);
          image_publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name + "/image_raw", 10);
          setEditorEnable(false);

          interective_marker_ = std::make_shared<InteMarker>("camera_info_plugins/marker", parent_frame_edit_->text().toStdString(), true);
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

  if (image_publisher_) {
    auto info = makeCameraInfoMsg();
    sensor_msgs::Image msg;
    msg.height = info.height;
    msg.width = info.width;
    msg.step = msg.width;
    msg.is_bigendian = true;
    msg.header = info.header;
    msg.encoding = "mono8";
    msg.data.resize(msg.height * msg.width);

    image_publisher_.publish(msg);
  }

  if (interective_marker_) {
    tf::Transform t = interective_marker_->getTransform();
    static tf::TransformBroadcaster br;
    auto stamped = tf::StampedTransform(t, ros::Time::now(), parent_frame_edit_->text().toStdString(), camera_frame_edit_->text().toStdString());
    br.sendTransform(stamped);
  }
}

void CameraInfoPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Topic", topic_edit_->text());
  config.mapSetValue("ParentFrame", parent_frame_edit_->text());
  config.mapSetValue("CameraFrame", camera_frame_edit_->text());
}

void CameraInfoPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  QString tmp_text;
  if (config.mapGetString("Topic", &tmp_text)) topic_edit_->setText(tmp_text);
  if (config.mapGetString("ParentFrame", &tmp_text)) parent_frame_edit_->setText(tmp_text);
  if (config.mapGetString("CameraFrame", &tmp_text)) camera_frame_edit_->setText(tmp_text);

  if (topic_edit_->text() == "") topic_edit_->setText("/camera_plugin");
  if (parent_frame_edit_->text() == "") parent_frame_edit_->setText("world");
  if (camera_frame_edit_->text() == "") camera_frame_edit_->setText("/camera_plugin");
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
  msg.header.frame_id = camera_frame_edit_->text().toStdString();
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
  ss << "K: ";
  for (auto k : msg.K)
    ss << k << " ";
  ss << "\n";
  return ss.str();
}

void CameraInfoPanel::setEditorEnable(bool enable)
{
  topic_edit_->setEnabled(enable);
  parent_frame_edit_->setEnabled(enable);
  camera_frame_edit_->setEnabled(enable);
}

}  // namespace camera_info_plugins


PLUGINLIB_EXPORT_CLASS(camera_info_plugins::CameraInfoPanel, rviz::Panel)
