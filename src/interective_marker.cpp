#include "interactive_marker.hpp"

namespace camera_info_plugins
{
InteMarker::InteMarker(const std::string& marker_name, const std::string& frame_id, bool fix_axis)
{
  transform.setIdentity();

  initMenu();
  server.reset(new InteractiveMarkerServer("menu", "", false));

  InteractiveMarker int_marker = makeMarker(marker_name, frame_id, fix_axis);
  server->insert(int_marker);

  server->setCallback(int_marker.name, boost::bind<void>(&InteMarker::actionCallback, this, _1));
  menu_handler.apply(*server, int_marker.name);
  server->applyChanges();
}

InteMarker::~InteMarker()
{
  server.reset();
}

Marker InteMarker::makeBox(const InteractiveMarker& msg)
{
  Marker marker;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.1;
  return marker;
}


void InteMarker::initMenu()
{
  // Reset menu
  menu_handler.insert("resetAll", boost::bind<void>(&InteMarker::resetAllCallback, this, _1));
  menu_handler.insert("resetRotate", boost::bind<void>(&InteMarker::resetRotateCallback, this, _1));
  menu_handler.insert("resetTranslate", boost::bind<void>(&InteMarker::resetTranslateCallback, this, _1));
}


void InteMarker::resetAllCallback(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO("Reset all");
  geometry_msgs::Pose pose;
  server->setPose(feedback->marker_name, pose);
  server->applyChanges();

  transform.setIdentity();
}

void InteMarker::resetRotateCallback(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO("Reset rotate");
  geometry_msgs::Pose pose = feedback->pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  server->setPose(feedback->marker_name, pose);
  server->applyChanges();

  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
}

void InteMarker::resetTranslateCallback(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO("Reset translate");
  geometry_msgs::Pose pose = feedback->pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  server->setPose(feedback->marker_name, pose);
  server->applyChanges();

  transform.setOrigin(tf::Vector3(0, 0, 0));
}

void InteMarker::actionCallback(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  geometry_msgs::Point p = feedback->pose.position;
  geometry_msgs::Quaternion q = feedback->pose.orientation;
  transform.setOrigin({p.x, p.y, p.z});
  transform.setRotation({q.x, q.y, q.z, q.w});
}

InteractiveMarker InteMarker::makeMarker(std::string name, const std::string& frame_id, bool fix_axis)
{
  InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.pose.orientation.w = 1;
  int_marker.scale = 1;
  int_marker.header.frame_id = frame_id;

  InteractiveMarkerControl control;

  if (fix_axis)
    control.orientation_mode = InteractiveMarkerControl::FIXED;

  {
    // 6軸の位置操作の機能を追加
    tf::quaternionTFToMsg(tf::Quaternion(1.0, 0.0, 0.0, 1.0).normalize(), control.orientation);
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    tf::quaternionTFToMsg(tf::Quaternion(0.0, 1.0, 0.0, 1.0).normalize(), control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    tf::quaternionTFToMsg(tf::Quaternion(0.0, 0.0, 1.0, 1.0).normalize(), control.orientation);
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }


  // control.markers.push_back(makeBox(int_marker));
  int_marker.controls.push_back(control);

  return int_marker;
}

}  // namespace camera_info_plugins