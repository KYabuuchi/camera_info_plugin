#pragma once
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>

namespace camera_info_plugins
{
using namespace visualization_msgs;
using namespace interactive_markers;

class InteMarker
{

public:
  InteMarker(const std::string& marker_name, const std::string& frame_id, bool fix_axis = false);

  ~InteMarker();

  tf::Transform getTransform() const { return transform; }


private:
  boost::shared_ptr<InteractiveMarkerServer> server;
  MenuHandler menu_handler;
  tf::Transform transform;

  Marker makeBox(const InteractiveMarker& msg);

  InteractiveMarker makeMarker(std::string name, const std::string& frame_id, bool fix_axis);

  void initMenu();


  void resetAllCallback(const InteractiveMarkerFeedbackConstPtr& feedback);
  void resetRotateCallback(const InteractiveMarkerFeedbackConstPtr& feedback);
  void resetTranslateCallback(const InteractiveMarkerFeedbackConstPtr& feedback);

  void actionCallback(const InteractiveMarkerFeedbackConstPtr& feedback);
};
}  // namespace camera_info_plugins