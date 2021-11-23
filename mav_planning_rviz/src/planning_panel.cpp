#include <stdio.h>
#include <functional>
#include <thread>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>
// #include <mav_planning_msgs/PlannerService.h>
#include <ros/names.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>

#include <mavros_msgs/SetMode.h>
#include <planner_msgs/SetString.h>
#include <planner_msgs/SetVector3.h>

#include "mav_planning_rviz/edit_button.h"
#include "mav_planning_rviz/goal_marker.h"
#include "mav_planning_rviz/planning_panel.h"
#include "mav_planning_rviz/pose_widget.h"

namespace mav_planning_rviz {

PlanningPanel::PlanningPanel(QWidget* parent) : rviz::Panel(parent), nh_(ros::NodeHandle()), interactive_markers_(nh_) {
  createLayout();
  goal_marker_ = std::make_shared<GoalMarker>(nh_);
}

void PlanningPanel::onInitialize() {
  interactive_markers_.initialize();
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this, std::placeholders::_1));

  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto& kv : pose_widget_map_) {
    mav_msgs::EigenTrajectoryPoint pose;
    kv.second->getPose(&pose);
    interactive_markers_.enableMarker(kv.first, pose);
  }
}

void PlanningPanel::createLayout() {
  QGridLayout* loadterrain_layout = new QGridLayout;
  // Input the namespace.
  loadterrain_layout->addWidget(new QLabel("Terrain Location:"), 0, 0);
  planner_name_editor_ = new QLineEdit;
  loadterrain_layout->addWidget(planner_name_editor_, 0, 1);
  terrain_align_checkbox_ = new QCheckBox("Disable Terrain Geolocation Alignment (Virtual Terrain)");
  loadterrain_layout->addWidget(terrain_align_checkbox_, 1, 0, 1, 2);
  load_terrain_button_ = new QPushButton("Load Terrain");
  loadterrain_layout->addWidget(load_terrain_button_, 2, 0, 2, 2);

  // Planner services and publications.
  QGridLayout* service_layout = new QGridLayout;
  planner_service_button_ = new QPushButton("Engage Planner");
  goal_altitude_editor_ = new QLineEdit;
  set_goal_button_ = new QPushButton("Update Goal");
  max_altitude_button_enable_ = new QPushButton("Enable Max altitude");
  max_altitude_button_disable_ = new QPushButton("Disable Max altitude");
  waypoint_button_ = new QPushButton("Disengage Planner");
  controller_button_ = new QPushButton("Send To Controller");
  service_layout->addWidget(new QLabel("Goal Altitude:"), 0, 0, 1, 1);
  service_layout->addWidget(goal_altitude_editor_, 0, 1, 1, 2);
  service_layout->addWidget(set_goal_button_, 1, 0, 1, 3);
  service_layout->addWidget(new QLabel("Max Altitude Constraints:"), 2, 0, 1, 1);
  service_layout->addWidget(max_altitude_button_enable_, 2, 1, 1, 1);
  service_layout->addWidget(max_altitude_button_disable_, 2, 2, 1, 1);
  service_layout->addWidget(planner_service_button_, 3, 0, 1, 3);
  service_layout->addWidget(waypoint_button_, 4, 0, 1, 3);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(loadterrain_layout);
  layout->addLayout(service_layout);
  setLayout(layout);

  // Hook up connections.
  connect(planner_name_editor_, SIGNAL(editingFinished()), this, SLOT(updatePlannerName()));
  connect(goal_altitude_editor_, SIGNAL(editingFinished()), this, SLOT(updateGoalAltitude()));
  connect(planner_service_button_, SIGNAL(released()), this, SLOT(callPlannerService()));
  connect(load_terrain_button_, SIGNAL(released()), this, SLOT(setPlannerName()));
  connect(set_goal_button_, SIGNAL(released()), this, SLOT(setGoalService()));
  connect(waypoint_button_, SIGNAL(released()), this, SLOT(publishWaypoint()));
  connect(max_altitude_button_enable_, SIGNAL(released()), this, SLOT(EnableMaxAltitude()));
  connect(max_altitude_button_disable_, SIGNAL(released()), this, SLOT(DisableMaxAltitude()));
  connect(controller_button_, SIGNAL(released()), this, SLOT(publishToController()));
  connect(terrain_align_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(terrainAlignmentStateChanged(int)));
}

void PlanningPanel::terrainAlignmentStateChanged(int state) {
  if (state == 0) {
    align_terrain_on_load_ = 1;
  } else {
    align_terrain_on_load_ = 0;
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setNamespace(const QString& new_namespace) {
  ROS_DEBUG_STREAM("Setting namespace from: " << namespace_.toStdString() << " to " << new_namespace.toStdString());
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(namespace_.toStdString() + "/waypoint", 1, false);
      controller_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(namespace_.toStdString() + "/command/pose", 1, false);
      odometry_sub_ = nh_.subscribe(namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
                                    &PlanningPanel::odometryCallback, this);
    }
  }
}

void PlanningPanel::updatePlannerName() {
  QString new_planner_name = planner_name_editor_->text();
  std::cout << "New Terrain name: " << new_planner_name.toStdString() << std::endl;
  if (new_planner_name != planner_name_) {
    planner_name_ = new_planner_name;
    Q_EMIT configChanged();
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setPlannerName() {
  std::cout << "[PlanningPanel] Loading new terrain:" << planner_name_.toStdString() << std::endl;
  // Load new environment using a service
  std::string service_name = "/terrain_planner/set_location";
  std::string new_planner_name = planner_name_.toStdString();
  bool align_terrain = align_terrain_on_load_;
  std::thread t([service_name, new_planner_name, align_terrain] {
    planner_msgs::SetString req;
    req.request.string = new_planner_name;
    req.request.align = align_terrain;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::updateGoalAltitude() { setGoalAltitude(goal_altitude_editor_->text()); }

void PlanningPanel::setGoalAltitude(const QString& new_goal_altitude) {
  if (new_goal_altitude != goal_altitude_value_) {
    goal_altitude_value_ = new_goal_altitude;
    Q_EMIT configChanged();
  }
}

// void PlanningPanel::updateOdometryTopic() { setOdometryTopic(odometry_topic_editor_->text()); }

// Set the topic name we are publishing to.
void PlanningPanel::setOdometryTopic(const QString& new_odometry_topic) {
  // Only take action if the name has changed.
  if (new_odometry_topic != odometry_topic_) {
    odometry_topic_ = new_odometry_topic;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      odometry_sub_ = nh_.subscribe(namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
                                    &PlanningPanel::odometryCallback, this);
    }
  }
}

void PlanningPanel::startEditing(const std::string& id) {
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
  interactive_markers_.enableSetPoseMarker(pose);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string& id) {
  if (currently_editing_ == id) {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
  ros::spinOnce();
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget) {
  pose_widget_map_[widget->id()] = widget;
  connect(widget, SIGNAL(poseUpdated(const std::string&, mav_msgs::EigenTrajectoryPoint&)), this,
          SLOT(widgetPoseUpdated(const std::string&, mav_msgs::EigenTrajectoryPoint&)));
}

void PlanningPanel::registerEditButton(EditButton* button) {
  edit_button_map_[button->id()] = button;
  connect(button, SIGNAL(startedEditing(const std::string&)), this, SLOT(startEditing(const std::string&)));
  connect(button, SIGNAL(finishedEditing(const std::string&)), this, SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("goal_altitude", goal_altitude_value_);
  config.mapSetValue("odometry_topic", odometry_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString topic;
  QString ns;
  if (config.mapGetString("planner_name", &planner_name_)) {
    planner_name_editor_->setText(planner_name_);
  }
  if (config.mapGetString("goal_altitude", &goal_altitude_value_)) {
    goal_altitude_editor_->setText(goal_altitude_value_);
  }
}

void PlanningPanel::updateInteractiveMarkerPose(const mav_msgs::EigenTrajectoryPoint& pose) {
  if (currently_editing_.empty()) {
    return;
  }
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string& id, mav_msgs::EigenTrajectoryPoint& pose) {
  if (currently_editing_ == id) {
    interactive_markers_.setPose(pose);
  }
  interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::callPlannerService() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([service_name] {
    mavros_msgs::SetMode req;
    req.request.custom_mode = "OFFBOARD";

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::callPublishPath() {
  std_srvs::Empty req;
  std::string service_name = namespace_.toStdString() + "/" + planner_name_.toStdString() + "/publish_path";
  try {
    if (!ros::service::call(service_name, req)) {
      ROS_WARN_STREAM("Couldn't call service: " << service_name);
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
  }
}

void PlanningPanel::publishWaypoint() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([service_name] {
    mavros_msgs::SetMode req;
    req.request.custom_mode = "AUTO.RTL";

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::EnableMaxAltitude() { setMaxAltitudeConstrant(true); }

void PlanningPanel::DisableMaxAltitude() { setMaxAltitudeConstrant(false); }

void PlanningPanel::setMaxAltitudeConstrant(bool set_constraint) {
  std::cout << "[PlanningPanel] Loading new terrain:" << planner_name_.toStdString() << std::endl;
  // Load new environment using a service
  std::string service_name = "/terrain_planner/set_max_altitude";
  std::string new_planner_name = "";
  bool align_terrain = set_constraint;
  std::thread t([service_name, new_planner_name, align_terrain] {
    planner_msgs::SetString req;
    req.request.string = new_planner_name;
    req.request.align = align_terrain;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setGoalService() {
  std::string service_name = "/terrain_planner/set_goal";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  try {
    goal_altitude = std::stod(goal_altitude_value_.toStdString());
    std::cout << "[PlanningPanel] Set Goal Altitude: " << goal_altitude << std::endl;
  } catch (const std::exception& e) {
    std::cout << "[PlanningPanel] Invalid Goal Altitude Set: " << e.what() << std::endl;
  }
  std::thread t([service_name, goal_pos, goal_altitude] {
    planner_msgs::SetVector3 req;
    req.request.vector.x = goal_pos(0);
    req.request.vector.y = goal_pos(1);
    // if ()
    req.request.vector.z = goal_altitude;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::publishToController() {
  mav_msgs::EigenTrajectoryPoint goal_point;
  goal_pose_widget_->getPose(&goal_point);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = vis_manager_->getFixedFrame().toStdString();
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point, &pose);

  ROS_DEBUG_STREAM("Publishing controller goal on " << controller_pub_.getTopic()
                                                    << " subscribers: " << controller_pub_.getNumSubscribers());

  controller_pub_.publish(pose);
}

void PlanningPanel::odometryCallback(const nav_msgs::Odometry& msg) {
  ROS_INFO_ONCE("Got odometry callback.");
  if (align_terrain_on_load_) {
    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(msg, &odometry);
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = odometry.position_W;
    point.orientation_W_B = odometry.orientation_W_B;
    pose_widget_map_["start"]->setPose(point);
    interactive_markers_.updateMarkerPose("start", point);
  }
}

}  // namespace mav_planning_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanel, rviz::Panel)
